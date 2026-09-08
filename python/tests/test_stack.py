"""End-to-end checks of the control stack against the firmware simulator."""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import pytest

from firmware_sim import FirmwareSimulator
from katzlab.config import (
    Binding,
    ControllerConfig,
    LaserConfig,
    MotionConfig,
    SystemConfig,
    load_system_config,
)
from katzlab.control import ControlLoop
from katzlab.drivers import LaserDriver, MotionDriver
from katzlab.drivers.laser import LaserError
from katzlab.drivers.motion import MotionError
from katzlab.input.base import ControllerFrame, EdgeTracker, InputSource, apply_deadzone_and_curve
from katzlab.transport import Link


@pytest.fixture
def sim():
    with FirmwareSimulator(terse=True) as simulator:
        yield simulator


@pytest.fixture
def link(sim):
    connection = Link(sim.port, 115200, read_timeout_s=3.0)
    connection.open(connect_timeout_s=0.2)
    sim.boot()          # the board prints its banner after the host connects
    yield connection
    connection.close()


# --------------------------------------------------------------------------
# transport
# --------------------------------------------------------------------------


def test_link_receives_boot_banner(link):
    line, _ = link.wait_for([r"COMBINED MOTION \+ LASER"], timeout_s=3.0)
    assert "COMBINED MOTION + LASER BUILD" in line


# --------------------------------------------------------------------------
# motion driver
# --------------------------------------------------------------------------


def test_motion_sync_and_relative_move(link, sim):
    motion = MotionDriver(link, MotionConfig())
    motion.sync(timeout_s=3.0)

    motion.move(linear_mm=0.1, rotation_deg=1.0, flexion_abs_deg=5.0)

    assert sim.received[-1].split() == ["0.1000", "1.0000", "5.000"]
    assert motion.state.linear_mm == pytest.approx(0.1, abs=1e-3)
    assert motion.state.flexion_tip_deg == pytest.approx(5.0, abs=1e-2)


def test_motion_position_comes_from_firmware_not_dead_reckoning(link, sim):
    motion = MotionDriver(link, MotionConfig())
    motion.sync(timeout_s=3.0)

    motion.move(linear_mm=0.1)
    # Something moves the carriage without Python knowing.
    sim.linear_mm = 42.0
    motion.move(linear_mm=0.1)

    assert motion.state.linear_mm == pytest.approx(42.1, abs=1e-3)


def test_motion_clamps_chunk_size(link, sim):
    motion = MotionDriver(link, MotionConfig())
    motion.sync(timeout_s=3.0)

    motion.move(linear_mm=999.0)

    sent_linear = float(sim.received[-1].split()[0])
    assert sent_linear == pytest.approx(MotionConfig().linear.max_chunk_mm)


def test_motion_clamps_against_travel_limits(link, sim):
    motion = MotionDriver(link, MotionConfig())
    motion.sync(timeout_s=3.0)
    sim.linear_mm = 177.0
    motion.move(linear_mm=0.0)          # sync Python's view of position

    motion.move(linear_mm=1.0)          # would exceed the 177 mm limit

    assert float(sim.received[-1].split()[0]) == pytest.approx(0.0, abs=1e-6)


def test_flexion_target_is_held_not_sprung_back(link):
    motion = MotionDriver(link, MotionConfig())
    motion.sync(timeout_s=3.0)

    motion.move(flexion_abs_deg=30.0)
    motion.move(linear_mm=0.05)         # stick released, flexion unspecified

    assert motion.flexion_target_deg == pytest.approx(30.0)


def test_flexion_target_clamped_to_envelope(link):
    motion = MotionDriver(link, MotionConfig())
    motion.sync(timeout_s=3.0)

    assert motion.set_flexion_target(999.0) == pytest.approx(270.0)
    assert motion.set_flexion_target(-999.0) == pytest.approx(-270.0)


def test_motion_reports_firmware_rejection(link, sim):
    motion = MotionDriver(link, MotionConfig())
    motion.sync(timeout_s=3.0)

    sim.reject_next = "Linear Range Error"
    with pytest.raises(MotionError, match="Linear Range Error"):
        motion.move(linear_mm=0.1)


def test_home_resets_flexion_target(link):
    motion = MotionDriver(link, MotionConfig())
    motion.sync(timeout_s=3.0)
    motion.move(flexion_abs_deg=100.0)

    motion.home(timeout_s=5.0)

    assert motion.flexion_target_deg == pytest.approx(0.0)
    assert motion.state.linear_mm == pytest.approx(0.0)


# --------------------------------------------------------------------------
# laser driver
# --------------------------------------------------------------------------


def test_laser_arm_sequence(link, sim):
    laser = LaserDriver(link, LaserConfig())
    laser.sync(timeout_s=3.0)

    state = laser.arm()

    assert sim.received[-2:] == ["e", "r"]
    assert state.enabled and state.ready and state.armed


def test_laser_refuses_fire_before_arming(link):
    laser = LaserDriver(link, LaserConfig())
    laser.sync(timeout_s=3.0)

    with pytest.raises(LaserError, match="not armed"):
        laser.fire()


def test_laser_fires_once_armed(link):
    laser = LaserDriver(link, LaserConfig())
    laser.sync(timeout_s=3.0)
    laser.arm()

    assert laser.fire().firing is True
    assert laser.pause().firing is False


def test_laser_stop_relocks(link):
    laser = LaserDriver(link, LaserConfig())
    laser.sync(timeout_s=3.0)
    laser.arm()

    state = laser.stop()

    assert not state.enabled and not state.ready and not state.firing


def test_shared_link_does_not_desync_drivers(link, sim):
    """Interleaved motion and laser traffic on one port must stay paired up.

    This is the single-board case: both drivers write to the same serial port,
    and the firmware reprints its motion prompt after laser commands too.
    """
    motion = MotionDriver(link, MotionConfig())
    motion.sync(timeout_s=3.0)
    laser = LaserDriver(link, LaserConfig())

    motion.move(linear_mm=0.1)
    laser.arm()
    motion.move(linear_mm=0.1)
    laser.fire()
    motion.move(linear_mm=0.1)

    assert motion.state.linear_mm == pytest.approx(0.3, abs=1e-3)
    assert laser.state.firing is True


# --------------------------------------------------------------------------
# input shaping
# --------------------------------------------------------------------------


def test_deadzone_suppresses_stick_drift():
    assert apply_deadzone_and_curve(0.05, 0.12, 0.35) == 0.0
    assert apply_deadzone_and_curve(-0.05, 0.12, 0.35) == 0.0


def test_curve_is_continuous_at_the_deadzone_edge():
    just_outside = apply_deadzone_and_curve(0.1201, 0.12, 0.35)
    assert 0.0 < just_outside < 0.01


def test_full_deflection_reaches_unity():
    assert apply_deadzone_and_curve(1.0, 0.12, 0.35) == pytest.approx(1.0)
    assert apply_deadzone_and_curve(-1.0, 0.12, 0.35) == pytest.approx(-1.0)


def test_edge_tracker_reports_press_and_release_once():
    edges = EdgeTracker()

    pressed, released = edges.update({"fire": True})
    assert pressed == {"fire"} and released == set()

    pressed, released = edges.update({"fire": True})
    assert pressed == set() and released == set()

    pressed, released = edges.update({"fire": False})
    assert pressed == set() and released == {"fire"}


# --------------------------------------------------------------------------
# control loop
# --------------------------------------------------------------------------


class ScriptedInput(InputSource):
    """Replays a fixed list of frames, then repeats the last one."""

    def __init__(self, frames: list[ControllerFrame]) -> None:
        self._frames = frames
        self._index = 0

    @property
    def name(self) -> str:
        return "scripted"

    def open(self) -> None:
        return

    def close(self) -> None:
        return

    def poll(self) -> ControllerFrame:
        frame = self._frames[min(self._index, len(self._frames) - 1)]
        self._index += 1
        return frame


def _controller_config() -> ControllerConfig:
    return ControllerConfig(
        name="test",
        guid="",
        axes={
            "linear": Binding(source="axis", index=0),
            "rotation": Binding(source="axis", index=2),
            "flexion": Binding(source="axis", index=3, invert=True),
        },
        buttons={
            name: Binding(source="button", index=i)
            for i, name in enumerate(
                ["enable_ready", "standby", "fire", "pause", "estop", "home"]
            )
        },
    )


def _loop(link, frames, **overrides):
    cfg = load_system_config()
    motion = MotionDriver(link, cfg.motion)
    motion.sync(timeout_s=3.0)
    laser = LaserDriver(link, cfg.laser)
    laser.status()      # banner already consumed by motion.sync(); ask outright
    return ControlLoop(cfg, ScriptedInput(frames), motion, laser), motion, laser


def _no_buttons() -> dict[str, bool]:
    return {
        name: False
        for name in ("enable_ready", "standby", "fire", "pause", "estop", "home")
    }


def test_stick_deflection_produces_motion(link, sim):
    frames = [ControllerFrame(axes={"linear": 1.0}, buttons=_no_buttons())]
    loop, motion, _ = _loop(link, frames)

    loop._service_motion(frames[0], elapsed_s=0.05)

    assert sim.received[-1].split()[0] != "0.0000"
    assert motion.state.linear_mm > 0.0


def test_centred_sticks_send_nothing(link, sim):
    frames = [ControllerFrame(axes={"linear": 0.0}, buttons=_no_buttons())]
    loop, _, _ = _loop(link, frames)
    before = len(sim.received)

    loop._service_motion(frames[0], elapsed_s=0.05)

    assert len(sim.received) == before


def test_estop_button_stops_and_relocks_the_laser(link, sim):
    buttons = _no_buttons()
    frames = [ControllerFrame(axes={}, buttons=buttons)]
    loop, _, laser = _loop(link, frames)
    laser.arm()
    assert laser.state.armed

    pressed = dict(buttons, estop=True)
    loop._handle_buttons(ControllerFrame(axes={}, buttons=pressed))

    # Assert the SAFETY OUTCOME, not which byte produced it. Whether e-stop
    # drives the ready contact is deliberately configurable -- on this rig that
    # contact arms rather than disarms -- so what must hold is that the laser is
    # disarmed, not firing, and the fire channels are parked.
    assert loop.estopped is True
    assert not laser.state.armed
    assert laser.state.firing is False
    assert sim.firing is False, "fire channels still asserted after an e-stop"
    assert sim.received[-1] in ("x", "p")


def test_estop_blocks_further_motion(link, sim):
    buttons = _no_buttons()
    loop, _, _ = _loop(link, [ControllerFrame(axes={}, buttons=buttons)])
    loop._handle_buttons(ControllerFrame(axes={}, buttons=dict(buttons, estop=True)))
    before = len(sim.received)

    frame = ControllerFrame(axes={"linear": 1.0}, buttons=buttons)
    if not loop.estopped:
        loop._service_motion(frame, 0.05)

    assert len(sim.received) == before


def test_standby_acknowledges_an_estop(link):
    buttons = _no_buttons()
    loop, _, _ = _loop(link, [ControllerFrame(axes={}, buttons=buttons)])
    loop._handle_buttons(ControllerFrame(axes={}, buttons=dict(buttons, estop=True)))
    assert loop.estopped

    loop._handle_buttons(ControllerFrame(axes={}, buttons=dict(buttons, standby=True)))

    assert loop.estopped is False


def test_controller_loss_triggers_estop(link, sim):
    buttons = _no_buttons()
    loop, _, laser = _loop(link, [ControllerFrame(axes={}, buttons=buttons)])
    laser.arm()

    loop._trigger_estop("controller disconnected")

    assert loop.estopped is True
    assert not laser.state.armed


def test_momentary_fire_press_and_release(link, sim):
    cfg = load_system_config()
    buttons = _no_buttons()
    loop, _, laser = _loop(link, [ControllerFrame(axes={}, buttons=buttons)])
    loop._handle_buttons(ControllerFrame(axes={}, buttons=dict(buttons, enable_ready=True)))
    assert laser.state.armed

    loop._handle_buttons(ControllerFrame(axes={}, buttons=dict(buttons, fire=True)))
    assert laser.state.firing is True

    # The pulse is held for a configured minimum, so a release inside that
    # window deliberately does NOT stop it.
    time.sleep(cfg.laser.min_fire_duration_s + 0.05)
    loop._handle_buttons(ControllerFrame(axes={}, buttons=buttons))
    assert laser.state.firing is False


def test_watchdog_expires_when_starved():
    from katzlab.control import Watchdog

    watchdog = Watchdog(timeout_s=0.05)
    assert not watchdog.expired
    time.sleep(0.08)
    assert watchdog.expired
    watchdog.feed()
    assert not watchdog.expired


# --------------------------------------------------------------------------
# rate-control timing
# --------------------------------------------------------------------------


def test_commanded_distance_tracks_wall_clock_not_idle_time(link, sim):
    """Rate control must integrate the whole cycle, including the blocking move.

    The bug this pins: _last_motion_at was stamped *after* move() returned, so
    the next cycle's `elapsed` measured only the gap between commands. Every
    command then asked for rate x idle_time instead of rate x cycle_time, and
    the carriage ran short by exactly the duty ratio -- which on real hardware
    read as "the linear axis barely moves".
    """
    sim.move_delay_s = 0.05          # firmware blocks for 50 ms per move
    cfg = load_system_config()
    motion = MotionDriver(link, cfg.motion)
    motion.sync(timeout_s=3.0)
    loop = ControlLoop(cfg, ScriptedInput([]), motion, None)

    # Mirror run()'s pacing: poll fast, issue a command once the motion period
    # has elapsed. Calling _service_motion in a tight loop instead would keep
    # `elapsed` at zero and measure nothing.
    frame = ControllerFrame(axes={"linear": 1.0}, buttons=_no_buttons())
    motion_period = 1.0 / cfg.motion.loop_hz
    began = time.monotonic()
    deadline = began + 0.8

    while time.monotonic() < deadline:
        elapsed = time.monotonic() - loop._last_motion_at
        if elapsed >= motion_period:
            loop._service_motion(frame, elapsed)
        time.sleep(0.005)

    duration = time.monotonic() - began

    achieved = sim.linear_mm / duration
    requested = cfg.motion.linear.max_rate_mm_s

    # Chunk clamping caps the top end, so this cannot reach `requested` exactly.
    # What it must not do is collapse to the duty-ratio fraction the bug caused.
    assert achieved > requested * 0.5, (
        f"only {achieved:.2f} mm/s of a requested {requested:.2f} -- "
        "rate integration is losing the blocking-move time again"
    )


def test_motion_clock_is_stamped_before_the_blocking_move(link, sim):
    sim.move_delay_s = 0.05
    cfg = load_system_config()
    motion = MotionDriver(link, cfg.motion)
    motion.sync(timeout_s=3.0)
    loop = ControlLoop(cfg, ScriptedInput([]), motion, None)

    frame = ControllerFrame(axes={"linear": 1.0}, buttons=_no_buttons())
    loop._service_motion(frame, 0.05)
    age = time.monotonic() - loop._last_motion_at

    # Stamped before a 50 ms move, so by the time it returns the stamp is >= 50 ms old.
    assert age >= 0.045, f"clock stamped after the move (age {age * 1000:.0f} ms)"


def test_flexion_deadband_stops_interrupting_the_servo(link, sim):
    """Small stick drift must not re-command the servo every cycle."""
    cfg = load_system_config()
    motion = MotionDriver(link, cfg.motion)
    motion.sync(timeout_s=3.0)

    deadband = cfg.motion.flexion.command_deadband_deg
    for _ in range(10):
        motion.move(flexion_abs_deg=motion.flexion_target_deg + deadband / 10)

    sent = [float(line.split()[2]) for line in sim.received if len(line.split()) == 3]
    distinct = len(set(sent))
    assert distinct <= 3, f"servo re-commanded {distinct} times inside the deadband"


def test_flexion_deadband_still_passes_real_movement_through(link, sim):
    cfg = load_system_config()
    motion = MotionDriver(link, cfg.motion)
    motion.sync(timeout_s=3.0)

    motion.move(flexion_abs_deg=45.0)

    assert float(sim.received[-1].split()[2]) == pytest.approx(45.0, abs=0.01)


# --------------------------------------------------------------------------
# laser arming determinism
# --------------------------------------------------------------------------


def test_arm_never_touches_the_fire_channels(link, sim):
    """Arming must not drive the fire relays.

    arm() used to send [x] first for a known starting state. [x] resets the
    FIRE channels too, and that transition is enough to make the machine fire --
    so pressing Enable/Ready fired the laser.
    """
    laser = LaserDriver(link, LaserConfig())
    laser.sync(timeout_s=3.0)
    laser.arm()                       # already armed
    before = len(sim.received)

    laser.arm()                       # arming again must still relock first

    issued = sim.received[before:]
    assert "x" not in issued, (
        f"arm() drove the fire channels: {issued}. [x] resets them, and that "
        "transition alone can make the machine fire."
    )
    assert issued[-2:] == ["e", "r"]


def test_arm_retries_when_ready_does_not_latch(link, sim):
    laser = LaserDriver(link, LaserConfig())
    laser.sync(timeout_s=3.0)

    original = sim._handle_laser
    failures = {"left": 1}

    def flaky(command: str) -> None:
        original(command)
        if command == "r" and failures["left"]:
            failures["left"] -= 1
            sim.ready = False         # first 'r' silently fails to latch

    sim._handle_laser = flaky
    state = laser.arm()

    assert state.ready is True
    assert sim.received.count("r") == 2, "did not retry the ready edge"


def test_arm_raises_with_guidance_when_it_never_latches(link, sim):
    laser = LaserDriver(link, LaserConfig())
    laser.sync(timeout_s=3.0)

    original = sim._handle_laser
    sim._handle_laser = lambda c: (original(c), setattr(sim, "ready", False))[0]

    with pytest.raises(LaserError, match="could not arm"):
        laser.arm()


def test_fire_still_refused_when_arming_failed(link, sim):
    laser = LaserDriver(link, LaserConfig())
    laser.sync(timeout_s=3.0)
    original = sim._handle_laser
    sim._handle_laser = lambda c: (original(c), setattr(sim, "ready", False))[0]

    with pytest.raises(LaserError):
        laser.arm()
    with pytest.raises(LaserError, match="not armed"):
        laser.fire()


# --------------------------------------------------------------------------
# laser commands must not disturb the control loop
# --------------------------------------------------------------------------


def test_blocking_laser_command_does_not_trip_the_watchdog(link, sim):
    """Arming blocks for most of a second. That must not read as a dead pad.

    The bug: arm() is three round trips plus the firmware's ready-edge settle
    plus both dwells. The loop cannot poll the controller while it waits, so the
    watchdog saw a gap and raised an E-STOP on every single arm.
    """
    cfg = load_system_config()
    motion = MotionDriver(link, cfg.motion)
    motion.sync(timeout_s=3.0)
    laser = LaserDriver(link, cfg.laser)
    laser.status()
    loop = ControlLoop(cfg, ScriptedInput([ControllerFrame(buttons=_no_buttons())]), motion, laser)

    time.sleep(cfg.safety.watchdog_timeout_s + 0.05)
    assert loop._watchdog.expired, "precondition: watchdog should be stale"

    loop._do_laser("arm")

    assert not loop._watchdog.expired, "watchdog left stale after a laser command"


def test_only_one_laser_action_per_frame(link, sim):
    """A frame with two edges must not run two blocking commands back to back.

    The second would act on an edge already a second old -- which is how an arm
    came out followed instantly by a fire the operator had let go of.
    """
    cfg = load_system_config()
    motion = MotionDriver(link, cfg.motion)
    motion.sync(timeout_s=3.0)
    laser = LaserDriver(link, cfg.laser)
    laser.status()
    loop = ControlLoop(cfg, ScriptedInput([ControllerFrame(buttons=_no_buttons())]), motion, laser)

    before = len([l for l in sim.received if l in ("e", "r", "f", "p", "s", "x")])
    both = dict(_no_buttons(), enable_ready=True, fire=True)
    loop._handle_buttons(ControllerFrame(axes={}, buttons=both))
    after = [l for l in sim.received if l in ("e", "r", "f", "p", "s", "x")][before:]

    assert "f" not in after, f"fire ran in the same frame as arm: {after}"


def test_fire_stops_even_if_the_release_edge_is_missed(link, sim):
    """A dropped frame must never leave the laser firing.

    Fire is level-driven, so a frame with the button not held and the laser
    firing always resolves to a stop, whether or not the release edge was seen.
    """
    cfg = load_system_config()
    motion = MotionDriver(link, cfg.motion)
    motion.sync(timeout_s=3.0)
    laser = LaserDriver(link, cfg.laser)
    laser.arm()
    loop = ControlLoop(cfg, ScriptedInput([ControllerFrame(buttons=_no_buttons())]), motion, laser)

    loop._handle_buttons(ControllerFrame(axes={}, buttons=dict(_no_buttons(), fire=True)))
    assert laser.state.firing is True

    # Wipe the edge tracker: the release transition is now unobservable.
    loop._edges.reset()
    time.sleep(cfg.laser.min_fire_duration_s + 0.05)
    loop._handle_buttons(ControllerFrame(axes={}, buttons=_no_buttons()))

    assert laser.state.firing is False, "laser kept firing after the button was released"


def test_standby_wins_over_arm_in_the_same_frame(link, sim):
    """Asked for two things at once, take the safer one."""
    cfg = load_system_config()
    motion = MotionDriver(link, cfg.motion)
    motion.sync(timeout_s=3.0)
    laser = LaserDriver(link, cfg.laser)
    laser.arm()
    loop = ControlLoop(cfg, ScriptedInput([ControllerFrame(buttons=_no_buttons())]), motion, laser)

    both = dict(_no_buttons(), enable_ready=True, standby=True)
    loop._handle_buttons(ControllerFrame(axes={}, buttons=both))

    assert laser.state.ready is False, "arm won over standby"


# --------------------------------------------------------------------------
# fire pulse duration
# --------------------------------------------------------------------------


def _fire_loop(link, cfg):
    motion = MotionDriver(link, cfg.motion)
    motion.sync(timeout_s=3.0)
    laser = LaserDriver(link, cfg.laser)
    laser.arm()
    loop = ControlLoop(cfg, ScriptedInput([ControllerFrame(buttons=_no_buttons())]), motion, laser)
    return loop, laser


def test_a_tap_still_produces_a_full_length_pulse(link, sim):
    """Releasing early must not cut the pulse short.

    A pulse only as long as the operator's finger registers as a press without
    completing a shot -- the machine "powers up but only goes half way".
    """
    cfg = load_system_config()
    loop, laser = _fire_loop(link, cfg)

    loop._handle_buttons(ControllerFrame(axes={}, buttons=dict(_no_buttons(), fire=True)))
    assert laser.state.firing is True

    # Button released immediately, well inside the minimum.
    loop._handle_buttons(ControllerFrame(axes={}, buttons=_no_buttons()))
    assert laser.state.firing is True, "pulse was cut short by an early release"

    time.sleep(cfg.laser.min_fire_duration_s + 0.05)
    loop._handle_buttons(ControllerFrame(axes={}, buttons=_no_buttons()))
    assert laser.state.firing is False, "pulse never ended after the minimum"


def test_holding_keeps_firing_past_the_minimum(link, sim):
    cfg = load_system_config()
    loop, laser = _fire_loop(link, cfg)
    held = dict(_no_buttons(), fire=True)

    loop._handle_buttons(ControllerFrame(axes={}, buttons=held))
    time.sleep(cfg.laser.min_fire_duration_s + 0.05)
    loop._handle_buttons(ControllerFrame(axes={}, buttons=held))

    assert laser.state.firing is True, "stopped firing while the button was held"


def test_fire_hits_a_hard_ceiling_even_while_held(link, sim):
    """The ceiling applies whatever the button says. A stuck button, a wedged
    frame, or a desynced state mirror must not mean an indefinite fire."""
    import dataclasses

    base = load_system_config()
    cfg = dataclasses.replace(
        base,
        laser=dataclasses.replace(
            base.laser, min_fire_duration_s=0.05, max_fire_duration_s=0.2
        ),
    )
    loop, laser = _fire_loop(link, cfg)

    held = dict(_no_buttons(), fire=True)
    loop._handle_buttons(ControllerFrame(axes={}, buttons=held))
    assert laser.state.firing is True

    time.sleep(0.25)
    loop._handle_buttons(ControllerFrame(axes={}, buttons=held))

    assert laser.state.firing is False, "fire ran past its ceiling while held"


def test_estop_clears_the_fire_latch(link, sim):
    cfg = load_system_config()
    loop, laser = _fire_loop(link, cfg)
    loop._handle_buttons(ControllerFrame(axes={}, buttons=dict(_no_buttons(), fire=True)))

    loop._trigger_estop("test")

    assert loop._fire_started_at is None
    assert laser.state.firing is False


def test_laser_commands_never_manufacture_button_presses(link, sim):
    """A laser command must not re-poll the pad and rewrite the edge tracker.

    Doing so let a single unlucky read invent a press on the following frame --
    standby firing on its own, and Fire acting like Enable/Ready.
    """
    cfg = load_system_config()
    motion = MotionDriver(link, cfg.motion)
    motion.sync(timeout_s=3.0)
    laser = LaserDriver(link, cfg.laser)
    laser.status()

    held = dict(_no_buttons(), enable_ready=True)
    loop = ControlLoop(cfg, ScriptedInput([ControllerFrame(buttons=held)]), motion, laser)

    loop._edges.update(held)          # operator is holding Enable/Ready
    loop._do_laser("arm")

    # The tracker must still show it held, so no new press edge appears.
    assert loop._edges.is_held("enable_ready"), (
        "edge tracker was rewritten during a laser command; a phantom press "
        "will fire on the next frame"
    )


def test_standby_can_be_told_not_to_drive_the_ready_contact(link, sim):
    """Both standby behaviours must work, whichever the config selects.

    On a machine whose ready contact ARMS on the level standby would drive,
    standby has to stop the fire channels and lock the software WITHOUT touching
    ready. This asserts that behaviour directly rather than reading it off the
    config default -- the default is a per-rig wiring decision and has already
    been flipped more than once.
    """
    import dataclasses

    base = load_system_config()
    cfg = dataclasses.replace(
        base, laser=dataclasses.replace(base.laser, standby_drives_ready=False)
    )

    laser = LaserDriver(link, cfg.laser)
    laser.arm()
    ready_before = sim.ready
    before = len(sim.received)

    laser.standby()

    issued = sim.received[before:]
    assert "s" not in issued and "x" not in issued, (
        f"standby drove the ready contact: {issued}"
    )
    assert sim.ready == ready_before, "ready contact changed on standby"
    assert laser.state.ready is False, "software still believes it is armed"
    assert sim.firing is False, "fire channels not parked"


def test_standby_does_drive_the_ready_contact_when_configured(link, sim):
    """The other half: with standby_drives_ready true, standby opens ready."""
    import dataclasses

    base = load_system_config()
    cfg = dataclasses.replace(
        base, laser=dataclasses.replace(base.laser, standby_drives_ready=True)
    )

    laser = LaserDriver(link, cfg.laser)
    laser.arm()
    before = len(sim.received)

    laser.standby()

    assert "s" in sim.received[before:], "standby did not open the ready contact"
    assert laser.state.ready is False


def test_arm_still_gets_its_open_edge_when_standby_does_not_drive_ready(link, sim):
    """Arming must not depend on the operator's standby behaviour.

    arm() used to call standby() to force the open half of its edge. When
    standby stopped driving the ready contact -- necessary on this rig, where
    holding it open ARMS the machine -- arming silently lost that edge, failed
    to latch, and every subsequent fire was refused for want of an armed
    machine. The two paths must be independent.
    """
    import dataclasses

    base = load_system_config()
    cfg = dataclasses.replace(
        base, laser=dataclasses.replace(base.laser, standby_drives_ready=False)
    )

    laser = LaserDriver(link, cfg.laser)
    before = len(sim.received)
    state = laser.arm()

    issued = sim.received[before:]
    assert "s" in issued, f"arm() lost its open edge: {issued}"
    assert issued[-2:] == ["e", "r"], f"arm() did not finish with e then r: {issued}"
    assert state.enabled and state.ready, "arm did not latch"


def test_fire_works_after_arming_with_standby_decoupled(link, sim):
    """The end-to-end case that broke: arm, then fire, with standby decoupled."""
    import dataclasses

    base = load_system_config()
    cfg = dataclasses.replace(
        base, laser=dataclasses.replace(base.laser, standby_drives_ready=False)
    )

    laser = LaserDriver(link, cfg.laser)
    laser.arm()
    state = laser.fire()

    assert state.firing is True, "fire failed after arming with standby decoupled"


def test_operator_standby_still_leaves_ready_alone(link, sim):
    """And the fix must not undo what it was for."""
    import dataclasses

    base = load_system_config()
    cfg = dataclasses.replace(
        base, laser=dataclasses.replace(base.laser, standby_drives_ready=False)
    )

    laser = LaserDriver(link, cfg.laser)
    laser.arm()
    ready_before = sim.ready
    before = len(sim.received)

    laser.standby()

    issued = sim.received[before:]
    assert "s" not in issued and "x" not in issued, (
        f"operator standby drove the ready contact: {issued}"
    )
    assert sim.ready == ready_before
    assert laser.state.ready is False
    assert sim.firing is False


# --------------------------------------------------------------------------
# e-stop vs standby: different trades on the ready contact
# --------------------------------------------------------------------------


def test_estop_drives_the_ready_contact_even_though_standby_does_not(link, sim):
    """E-stop is the exception to leaving the ready line alone.

    Standby avoids it because on this wiring driving it can arm rather than
    disarm. E-stop takes that chance anyway: leaving a machine armed after
    someone hit the panic button is the worse of the two failures.
    """
    import dataclasses

    base = load_system_config()
    cfg = dataclasses.replace(
        base,
        laser=dataclasses.replace(
            base.laser, standby_drives_ready=False, estop_drives_ready=True
        ),
    )
    motion = MotionDriver(link, cfg.motion)
    motion.sync(timeout_s=3.0)
    laser = LaserDriver(link, cfg.laser)
    laser.arm()
    loop = ControlLoop(cfg, ScriptedInput([ControllerFrame(buttons=_no_buttons())]), motion, laser)

    before = len(sim.received)
    loop._trigger_estop("test")
    issued = sim.received[before:]

    assert "x" in issued, f"e-stop did not drive the ready contact: {issued}"
    assert sim.ready is False, "machine left ready after an e-stop"
    assert sim.firing is False
    assert not laser.state.armed


def test_standby_still_leaves_the_ready_contact_alone(link, sim):
    """The e-stop change must not leak into Standby."""
    import dataclasses

    base = load_system_config()
    cfg = dataclasses.replace(
        base,
        laser=dataclasses.replace(
            base.laser, standby_drives_ready=False, estop_drives_ready=True
        ),
    )
    laser = LaserDriver(link, cfg.laser)
    laser.arm()
    ready_before = sim.ready
    before = len(sim.received)

    laser.standby()

    issued = sim.received[before:]
    assert "s" not in issued and "x" not in issued, (
        f"standby drove the ready contact: {issued}"
    )
    assert sim.ready == ready_before


def test_estop_can_be_told_to_leave_ready_alone_too(link, sim):
    import dataclasses

    base = load_system_config()
    cfg = dataclasses.replace(
        base,
        laser=dataclasses.replace(
            base.laser, standby_drives_ready=False, estop_drives_ready=False
        ),
    )
    motion = MotionDriver(link, cfg.motion)
    motion.sync(timeout_s=3.0)
    laser = LaserDriver(link, cfg.laser)
    laser.arm()
    loop = ControlLoop(cfg, ScriptedInput([ControllerFrame(buttons=_no_buttons())]), motion, laser)

    before = len(sim.received)
    loop._trigger_estop("test")

    assert "x" not in sim.received[before:]
    assert sim.firing is False, "fire channels must be parked regardless"
    assert not laser.state.armed, "software must lock out regardless"
