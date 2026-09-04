"""Velocity-mode driver and control-loop behaviour."""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import pytest

from katzlab.config import load_system_config
from katzlab.control import ControlLoop
from katzlab.drivers import VelocityMotionDriver
from katzlab.input.base import ControllerFrame, InputSource
from katzlab.transport import Link
from velocity_sim import VelocitySimulator

BUTTONS = {n: False for n in ("enable_ready", "standby", "fire", "pause", "estop", "home")}


class Null(InputSource):
    name = "null"
    def open(self): pass
    def close(self): pass
    def poll(self): return ControllerFrame()


@pytest.fixture
def sim():
    with VelocitySimulator() as s:
        yield s


@pytest.fixture
def driver(sim):
    link = Link(sim.port, 115200, read_timeout_s=3.0)
    link.open(connect_timeout_s=0.2)
    sim.boot()
    cfg = load_system_config()
    d = VelocityMotionDriver(link, cfg.motion)
    d.sync(timeout_s=3.0)
    yield d
    link.close()


def test_sync_recognises_the_velocity_firmware(driver):
    assert driver.state is not None


def test_velocity_command_is_not_blocking(driver):
    """The whole point: setting a speed must return immediately."""
    began = time.monotonic()
    driver.set_velocity(linear_mm_s=5.0)
    assert time.monotonic() - began < 0.05


def test_speed_is_proportional_to_the_command(driver, sim):
    """Derive the test speeds from the configured ceiling.

    Hardcoded magnitudes silently stop testing anything the moment the ceiling
    is tuned below them -- both commands clamp to the same value and the
    assertion compares a number against itself.
    """
    ceiling = load_system_config().motion.linear.max_rate_mm_s

    driver.set_velocity(linear_mm_s=ceiling * 0.4)
    time.sleep(0.35)
    slower = sim.current_linear

    driver.set_velocity(linear_mm_s=ceiling)
    time.sleep(0.35)
    faster = sim.current_linear

    assert faster > slower * 1.5, f"{slower:.2f} -> {faster:.2f} is not proportional"


def test_linear_axis_has_a_linear_response_curve():
    """expo 0 means stick position maps straight through to speed."""
    from katzlab.input.base import apply_deadzone_and_curve

    cfg = load_system_config()
    expo = cfg.input.expo_for("linear")
    assert expo == 0.0, "linear axis should not be shaped"

    deadzone = cfg.input.deadzone
    for deflection in (0.3, 0.5, 0.7, 0.9):
        shaped = apply_deadzone_and_curve(deflection, deadzone, expo)
        # Above the deadzone the mapping is the straight rescale, nothing more.
        expected = (deflection - deadzone) / (1.0 - deadzone)
        assert shaped == pytest.approx(expected, abs=1e-6)


def test_velocity_is_clamped_to_the_configured_ceiling(driver, sim):
    cfg = load_system_config()
    driver.set_velocity(linear_mm_s=999.0)
    sent = [l for l in sim.received if l.startswith("L")][-1]
    assert float(sent[1:]) == pytest.approx(cfg.motion.linear.max_rate_mm_s)


def test_motion_is_continuous_not_stop_start(driver, sim):
    """Velocity must stay up between commands rather than returning to zero."""
    driver.set_velocity(linear_mm_s=4.0)
    time.sleep(0.4)

    samples = []
    for _ in range(6):
        samples.append(sim.current_linear)
        time.sleep(0.05)

    assert all(v > 1.0 for v in samples), f"velocity collapsed between commands: {samples}"


def test_stop_takes_effect_immediately(driver, sim):
    driver.set_velocity(linear_mm_s=6.0)
    time.sleep(0.3)
    assert sim.current_linear > 1.0

    driver.stop()
    time.sleep(0.1)
    assert sim.current_linear == pytest.approx(0.0, abs=0.01)


def test_position_arrives_as_telemetry(driver, sim):
    driver.set_velocity(linear_mm_s=4.0)
    time.sleep(0.5)
    driver.stop()
    assert driver.state.linear_mm > 0.1


def test_velocity_is_resent_every_call_to_feed_the_watchdog(driver, sim):
    """An unchanged velocity must STILL be sent.

    This test previously asserted the opposite, and that is exactly what broke
    on hardware: the firmware reads a quiet link as a dead host and ramps to a
    stop, so holding the stick steady stalled the axis after the 400 ms timeout.
    Suppressing redundant writes and having a host watchdog are incompatible.
    """
    driver.set_velocity(linear_mm_s=3.0)
    before = len([l for l in sim.received if l.startswith("L")])

    for _ in range(5):
        driver.set_velocity(linear_mm_s=3.0)

    after = len([l for l in sim.received if l.startswith("L")])
    assert after == before + 5, "unchanged velocity was suppressed; watchdog will trip"


def test_held_stick_keeps_moving_past_the_watchdog_timeout(driver, sim):
    """Hold full deflection longer than the firmware timeout and keep going."""
    cfg = load_system_config()
    loop = ControlLoop(cfg, Null(), driver, None)
    frame = ControllerFrame(axes={"linear": 1.0}, buttons=BUTTONS)

    began = time.monotonic()
    while time.monotonic() - began < 0.9:        # well past the 400 ms watchdog
        loop._service_motion(frame, 0.016)
        time.sleep(0.016)

    assert sim.current_linear > 1.0, (
        f"velocity collapsed to {sim.current_linear:.2f} while the stick was held"
    )
    assert sim.linear_mm > 0.5


def test_flexion_is_not_resent_inside_its_deadband(driver, sim):
    """Flexion is the one thing that must NOT be spammed.

    Re-commanding an absolute servo target restarts its acceleration ramp. The
    velocity lines feed the watchdog, so flexion does not need to.
    """
    driver.set_velocity(linear_mm_s=0.0, flexion_abs_deg=10.0)
    before = len([l for l in sim.received if l.startswith("F")])
    for _ in range(5):
        driver.set_velocity(linear_mm_s=0.0, flexion_abs_deg=10.01)
    after = len([l for l in sim.received if l.startswith("F")])
    assert after == before


def test_home_is_abortable(driver, sim):
    driver.set_velocity(linear_mm_s=6.0)
    time.sleep(0.5)
    driver.stop()

    calls = {"n": 0}
    def abort():
        calls["n"] += 1
        return calls["n"] > 3

    driver.home(timeout_s=5.0, should_abort=abort)
    assert sim.homing is False
    assert sim.current_linear == pytest.approx(0.0, abs=0.01)


def test_control_loop_maps_stick_straight_to_velocity(driver, sim):
    cfg = load_system_config()
    loop = ControlLoop(cfg, Null(), driver, None)

    loop._service_motion(
        ControllerFrame(axes={"linear": 0.5}, buttons=BUTTONS), 0.02
    )
    sent = [l for l in sim.received if l.startswith("L")][-1]
    assert float(sent[1:]) == pytest.approx(cfg.motion.linear.max_rate_mm_s * 0.5, abs=0.01)


def test_estop_actively_stops_motion_in_velocity_mode(driver, sim):
    cfg = load_system_config()
    loop = ControlLoop(cfg, Null(), driver, None)
    driver.set_velocity(linear_mm_s=6.0)
    time.sleep(0.3)

    loop._handle_buttons(ControllerFrame(axes={}, buttons=dict(BUTTONS, estop=True)))
    time.sleep(0.1)

    assert loop.estopped is True
    assert sim.current_linear == pytest.approx(0.0, abs=0.01)


def test_flexion_deadband_is_sized_against_the_flexion_rate():
    """The servo command interval is deadband / rate, and it has to stay small.

    This is the mismatch that caused it twice: the deadband was tuned at one
    rate, the rate was later lowered for feel, and nobody rescaled the deadband.
    A 4 deg deadband is continuous at 30 deg/s and a visible hop every half
    second at 8 deg/s -- same config, completely different behaviour.
    """
    flexion = load_system_config().motion.flexion

    interval_s = flexion.command_deadband_deg / flexion.max_rate_deg_s
    assert interval_s <= 0.1, (
        f"servo re-commanded only every {interval_s * 1000:.0f} ms at full "
        f"stick ({flexion.command_deadband_deg} deg / {flexion.max_rate_deg_s} "
        "deg/s) -- that reads as discrete hops, not motion"
    )

    assert flexion.firmware_deadband_deg <= flexion.command_deadband_deg, (
        "the firmware deadband is the fine gate and must not be coarser than "
        "the host's"
    )


def test_flexion_deadband_is_pushed_to_the_firmware_on_sync(driver, sim):
    cfg = load_system_config()
    sent = [l for l in sim.received if l.startswith("D")]
    assert sent, "flexion deadband was never sent to the firmware"
    assert float(sent[-1][1:]) == pytest.approx(
        cfg.motion.flexion.firmware_deadband_deg, abs=1e-3
    )


def test_motion_does_not_eat_the_lasers_replies(driver, sim):
    """On a shared port the motion driver must consume only its own telemetry.

    set_velocity() returns self.state, which drains the link, and it runs on
    every control tick. Draining everything meant the laser's command replies
    were swallowed 60 times a second -- so its commands timed out and its state
    mirror went stale, which is indistinguishable from the laser misbehaving.
    """
    from katzlab.drivers import LaserDriver

    cfg = load_system_config()
    laser = LaserDriver(driver._link, cfg.laser)

    # Laser reply lands in the queue, then a motion tick runs before it is read.
    driver._link.write_line("?")
    time.sleep(0.5)
    driver.set_velocity(linear_mm_s=1.0)

    remaining = "\n".join(driver._link.drain())
    assert "Pin levels" in remaining or "enabled/ready" in remaining, (
        "motion drained the laser's reply; the laser will time out"
    )


def test_laser_commands_survive_a_busy_motion_loop(driver, sim):
    from katzlab.drivers import LaserDriver

    cfg = load_system_config()
    laser = LaserDriver(driver._link, cfg.laser)
    laser.status()

    for _ in range(10):
        driver.set_velocity(linear_mm_s=2.0)
    state = laser.arm()

    assert state.enabled and state.ready, "arm failed while motion was streaming"
