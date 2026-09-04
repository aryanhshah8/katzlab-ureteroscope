"""The control loop: controller frames in, firmware commands out.

Button map, as specified for the rig::

    left stick X          linear travel          (rate)
    right stick X         rotation               (rate)
    right stick Y         flexion                (rate, integrated to absolute)
    left secondary  (LB)  Enable + Ready
    left primary    (LT)  Standby
    right secondary (RB)  Fire
    right primary   (RT)  Pause
    X  (face, bottom)     E-stop
    H  (face, right)      Home

A note on responsiveness that the design has to live with: the 3-DOF firmware
executes each move to completion before reading serial again, so a move in
flight cannot be cancelled. Two things bound the damage. Chunk sizes are capped
in system.yaml so no single blocking move lasts more than roughly a tenth of a
second, and the loop simply stops issuing new commands the instant an e-stop is
raised. Worst-case e-stop latency is therefore about one chunk, not unbounded --
but it is not zero, and on a single shared link a laser stop queues behind an
in-flight motion command. Removing that limit needs the firmware rewrite
(approach 1), not more Python.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field

from .config import SystemConfig
from .drivers import LaserDriver, LaserState, MotionDriver, MotionState
from .drivers.laser import LaserError
from .drivers.motion import MotionError
from .input import ControllerFrame, EdgeTracker, InputSource, InputUnavailable
from .recorder import SessionRecorder

log = logging.getLogger(__name__)


# Home traverses the whole travel. Requiring a deliberate hold stops a stray
# press from launching it.
HOME_HOLD_S = 0.6

# Minimum gap between two identical laser commands. Long enough that a held or
# bouncing button cannot re-trigger, short enough to never feel laggy.
LASER_ACTION_COOLDOWN_S = 0.5


class EmergencyStop(RuntimeError):
    """Raised to unwind the loop after an e-stop."""


@dataclass
class LoopStats:
    frames: int = 0
    motion_commands: int = 0
    laser_commands: int = 0
    estops: int = 0
    errors: int = 0
    started_at: float = field(default_factory=time.monotonic)

    @property
    def uptime_s(self) -> float:
        return time.monotonic() - self.started_at


class Watchdog:
    """Trips when controller frames stop arriving.

    A dropped Bluetooth link or an unplugged cable must not leave the rig
    holding its last commanded rate, so a stale frame is treated the same as a
    centred stick.
    """

    def __init__(self, timeout_s: float) -> None:
        self._timeout_s = timeout_s
        self._last_ok = time.monotonic()

    def feed(self) -> None:
        self._last_ok = time.monotonic()

    @property
    def expired(self) -> bool:
        return (time.monotonic() - self._last_ok) > self._timeout_s

    @property
    def age_s(self) -> float:
        return time.monotonic() - self._last_ok


class ControlLoop:
    """Binds an input source to the motion and laser drivers."""

    def __init__(
        self,
        cfg: SystemConfig,
        source: InputSource,
        motion: MotionDriver,
        laser: LaserDriver | None,
    ) -> None:
        self._cfg = cfg
        self._source = source
        self._motion = motion
        self._laser = laser

        self._edges = EdgeTracker()
        self._watchdog = Watchdog(cfg.safety.watchdog_timeout_s)
        self._stats = LoopStats()

        self._running = False
        self._estopped = False
        self._last_motion_at = time.monotonic()
        self._latched_firing = False
        self._home_hold_started: float | None = None
        self._fire_started_at: float | None = None
        self._laser_action_at: dict[str, float] = {}
        self.debug_buttons = False
        self.recorder: SessionRecorder | None = None

    @property
    def stats(self) -> LoopStats:
        return self._stats

    @property
    def estopped(self) -> bool:
        return self._estopped

    # -- main loop ---------------------------------------------------------

    def run(self, status_callback=None) -> None:
        """Run until interrupted. ``status_callback`` receives a status dict."""
        self._running = True
        poll_period = 1.0 / max(1.0, self._cfg.input.poll_hz)
        motion_period = 1.0 / max(1.0, self._cfg.motion.loop_hz)

        log.info("control loop starting (poll %.0f Hz, motion %.0f Hz)",
                 self._cfg.input.poll_hz, self._cfg.motion.loop_hz)

        try:
            while self._running:
                cycle_start = time.monotonic()

                frame = self._poll()
                self._stats.frames += 1

                if frame.connected:
                    self._watchdog.feed()
                    self._handle_buttons(frame)
                elif self._cfg.safety.estop_on_controller_loss:
                    self._trigger_estop("controller disconnected")

                if self._watchdog.expired and self._cfg.safety.estop_on_controller_loss:
                    self._trigger_estop(
                        f"no controller frame for {self._watchdog.age_s:.2f}s"
                    )

                if not self._estopped and frame.connected:
                    elapsed = time.monotonic() - self._last_motion_at
                    # Velocity mode never blocks, so there is nothing to pace:
                    # run it every tick for the tightest stick response.
                    if self._velocity_mode or elapsed >= motion_period:
                        self._service_motion(frame, elapsed)

                if self.recorder is not None:
                    self.recorder.sample(
                        self._motion.state,
                        self._laser.state if self._laser else None,
                        estopped=self._estopped,
                    )

                if status_callback is not None:
                    status_callback(self.status())

                slack = poll_period - (time.monotonic() - cycle_start)
                if slack > 0:
                    time.sleep(slack)

        except KeyboardInterrupt:
            log.info("interrupted by operator")
        finally:
            self.shutdown()

    def stop(self) -> None:
        self._running = False

    # -- input -------------------------------------------------------------

    def _poll(self) -> ControllerFrame:
        try:
            return self._source.poll()
        except InputUnavailable as exc:
            log.error("input source failed: %s", exc)
            return ControllerFrame(connected=False)

    # -- buttons -----------------------------------------------------------

    def _handle_buttons(self, frame: ControllerFrame) -> None:
        pressed, released = self._edges.update(frame.buttons)

        # With --debug-buttons, record exactly what arrived and what the loop
        # decided. Arguing about whether Fire is behaving like Enable/Ready is
        # pointless when the frame itself can just be printed.
        if self.debug_buttons and (pressed or released):
            down = sorted(n for n, v in frame.buttons.items() if v)
            log.info(
                "BUTTONS  down=%s  pressed=%s  released=%s",
                down or "-", sorted(pressed) or "-", sorted(released) or "-",
            )

        # E-stop is checked first and unconditionally, before anything that
        # could raise and skip it.
        if "estop" in pressed:
            self._trigger_estop("X pressed")
            return

        if self._estopped:
            # Standby is the acknowledge-and-clear gesture: it puts the laser
            # in a known safe state, so it is the one thing allowed to reset.
            if "standby" in pressed:
                self._clear_estop()
            return

        try:
            if self._home_hold_started is not None and not self._edges.is_held("home"):
                self._home_hold_started = None
            elif "home" in pressed:
                self._home_hold_started = time.monotonic()
            elif (
                self._home_hold_started is not None
                and time.monotonic() - self._home_hold_started >= HOME_HOLD_S
            ):
                self._home_hold_started = None
                self._do_home()

            # At most ONE laser action per frame, most-restrictive first.
            #
            # These were separate `if`s, so a frame carrying two edges ran both
            # in sequence -- and because each one blocks for the better part of
            # a second, the second acted on an edge that was already stale. An
            # arm followed instantly by a fire in the log is exactly that: two
            # actions from one frame, the fire landing long after the operator
            # let go. Standby and pause win over arm and fire deliberately; if
            # the operator is asking for two things at once, take the safer one.
            if "standby" in pressed:
                if self.debug_buttons:
                    log.info("  -> dispatching STANDBY")
                self._do_laser("standby")
            elif "pause" in pressed:
                self._do_laser("pause")
            elif "enable_ready" in pressed:
                if self.debug_buttons:
                    log.info("  -> dispatching ARM")
                self._do_laser("arm")
            else:
                if self.debug_buttons and pressed:
                    log.info("  -> dispatching FIRE handler")
                self._handle_fire(frame, pressed, released)
        except (MotionError, LaserError) as exc:
            self._stats.errors += 1
            log.error("command failed: %s", exc)

    def _handle_fire(
        self, frame: ControllerFrame, pressed: set[str], released: set[str]
    ) -> None:
        if self._laser is None:
            return

        if self._cfg.laser.fire_mode == "momentary":
            # Hold to fire, driven by the button's LEVEL rather than its edges.
            #
            # Edges can be missed: a laser command blocks for the better part of
            # a second, and anything that re-baselines the edge tracker during
            # that window swallows the transition. For a start button that is
            # merely annoying; for a stop it means the laser keeps firing after
            # the operator has let go. Comparing held-state against firing-state
            # is idempotent -- a dropped frame costs latency, never a missed
            # stop, because the next frame reaches the same conclusion.
            held = frame.button("fire")
            firing = self._laser.state.firing
            now = time.monotonic()

            # START on the press EDGE, STOP on the level.
            #
            # Level-driven starting retried the fire every single frame while
            # the button was held. With the laser disarmed that is dozens of
            # refusals a second -- the wall of "refusing to fire" in the log --
            # and it drowns out everything useful. Stopping stays level-driven,
            # because a missed release must never leave the laser firing.
            if "fire" in pressed and not firing:
                self._do_laser("fire")
                self._fire_started_at = now
                return

            if not firing:
                return

            elapsed = now - (self._fire_started_at or now)

            # Hard ceiling first, so it applies whatever the button says and
            # whatever the state mirror believes.
            if elapsed >= self._cfg.laser.max_fire_duration_s:
                log.warning(
                    "fire hit the %.1fs ceiling; releasing",
                    self._cfg.laser.max_fire_duration_s,
                )
                self._do_laser("pause")
                self._fire_started_at = None
                return

            # Below the minimum, keep the contacts closed even though the button
            # is already up. A pulse shorter than the machine needs registers as
            # a press without completing a shot.
            if not held and elapsed >= self._cfg.laser.min_fire_duration_s:
                self._do_laser("pause")
                self._fire_started_at = None
            return

        # Latched: press toggles.
        if "fire" in pressed:
            if self._latched_firing:
                self._do_laser("pause")
                self._latched_firing = False
            else:
                self._do_laser("fire")
                self._latched_firing = True

    def _do_laser(self, action: str) -> LaserState | None:
        """Run one laser command, then re-sync the loop's sense of time.

        These block. arm() alone is three serial round trips plus the firmware's
        200 ms ready-edge settle plus both dwell timers -- comfortably longer
        than the controller watchdog. Without feeding the watchdog afterwards,
        arming the laser TRIPS AN E-STOP every time, because the loop could not
        poll the pad while it was waiting. That is what "E-STOP: no controller
        frame for 0.51s" immediately after an arm was.
        """
        if self._laser is None:
            log.warning("laser action %r ignored: no laser link configured", action)
            return None

        # Cooldown per action. The log showed arm and standby repeating about
        # once a second for as long as a button was down: each blocking command
        # is long enough that the edge tracker gets re-baselined mid-press, so a
        # held button reads as a fresh press again and again. Rate-limiting the
        # ACTION is robust to however the edges are being seen.
        now = time.monotonic()
        last = self._laser_action_at.get(action, 0.0)
        if now - last < LASER_ACTION_COOLDOWN_S:
            return self._laser.state
        self._laser_action_at[action] = now

        if self.recorder is not None:
            self.recorder.mark(f"laser_{action}")

        state = getattr(self._laser, action)()
        self._stats.laser_commands += 1
        log.info("laser %s -> %s", action, state.describe())

        # The controller was never gone; we were busy. Re-baseline both the
        # watchdog and the motion clock so neither reads the blocked interval as
        # a fault or as travel the operator asked for.
        self._watchdog.feed()
        self._last_motion_at = time.monotonic()

        # NO edge re-baselining here, deliberately.
        #
        # Re-polling the pad mid-command and overwriting the tracker was meant
        # to discard stale edges. It did something worse: a single poll that
        # catches a held button as momentarily not-held makes the NEXT frame
        # look like a fresh press. That manufactures commands the operator never
        # gave -- standby appearing on its own, and Fire behaving like
        # Enable/Ready. A phantom press is far worse than a late one, and the
        # per-action cooldown above already handles staleness without inventing
        # anything.
        return state

    def _do_home(self) -> MotionState:
        """Home all axes, polling for an abort between chunks.

        Home is the longest move on the rig. Driving it from the driver in
        chunks means the pad stays live throughout, so E-stop still works --
        which it did not when this was one blocking firmware command.
        """
        log.info("home requested -- press X to abort")
        if self.recorder is not None:
            self.recorder.mark("home")

        def should_abort() -> bool:
            frame = self._poll()
            if not frame.connected:
                log.warning("controller lost during home")
                return True
            if frame.button("estop"):
                log.critical("home aborted by E-stop")
                self._trigger_estop("X pressed during home")
                return True
            return False

        state = self._motion.home(should_abort=should_abort)
        self._stats.motion_commands += 1
        return state

    # -- motion ------------------------------------------------------------

    @property
    def _velocity_mode(self) -> bool:
        return getattr(self._motion, "is_velocity_mode", False)

    def _service_velocity(self, frame: ControllerFrame, elapsed_s: float) -> None:
        """Push stick deflection straight through as target velocities.

        No integration, no chunking: the stick position IS the commanded speed,
        which is why this is both smooth and proportional. Flexion still
        integrates, because it is an absolute servo angle rather than a rate.
        """
        motion = self._cfg.motion
        elapsed_s = min(elapsed_s, 0.25)

        linear = frame.axis("linear") * motion.linear.max_rate_mm_s
        rotation = frame.axis("rotation") * motion.rotation.max_rate_deg_s
        flexion_target = (
            self._motion.flexion_target_deg
            + frame.axis("flexion") * motion.flexion.max_rate_deg_s * elapsed_s
        )

        self._motion.set_velocity(
            linear_mm_s=linear,
            rotation_deg_s=rotation,
            flexion_abs_deg=flexion_target,
        )
        self._stats.motion_commands += 1
        self._warn_on_flexion(self._motion.flexion_target_deg)

    def _service_motion(self, frame: ControllerFrame, elapsed_s: float) -> None:
        """Integrate stick deflection into one relative move command."""
        if self._velocity_mode:
            self._last_motion_at = time.monotonic()
            self._service_velocity(frame, elapsed_s)
            return

        motion = self._cfg.motion

        # Clamp the integration window. If the loop was stalled -- a long home,
        # a serial timeout -- do not turn that whole gap into one lunge the
        # operator never asked for.
        elapsed_s = min(elapsed_s, 0.25)

        linear_mm = frame.axis("linear") * motion.linear.max_rate_mm_s * elapsed_s
        rotation_deg = frame.axis("rotation") * motion.rotation.max_rate_deg_s * elapsed_s
        flexion_step = frame.axis("flexion") * motion.flexion.max_rate_deg_s * elapsed_s

        flexion_target = self._motion.flexion_target_deg + flexion_step
        flexion_changed = abs(flexion_step) > 1e-4

        # One firmware step is 0.0025 mm; anything below that is a wasted round
        # trip that would only add latency to the next real command.
        if (
            abs(linear_mm) < 0.0025
            and abs(rotation_deg) < 0.01
            and not flexion_changed
        ):
            self._last_motion_at = time.monotonic()
            return

        # Stamp the clock BEFORE the move, not after. move() blocks for the
        # whole duration of the motion, so stamping afterwards made the next
        # cycle's `elapsed` measure only the idle gap between commands rather
        # than the full cycle -- and the commanded distance came up short by
        # exactly the duty ratio. Rate control has to integrate wall-clock time.
        self._last_motion_at = time.monotonic()

        try:
            self._motion.move(
                linear_mm=linear_mm,
                rotation_deg=rotation_deg,
                flexion_abs_deg=flexion_target,
            )
            self._stats.motion_commands += 1
        except MotionError as exc:
            self._stats.errors += 1
            log.error("motion command failed: %s", exc)

        self._warn_on_flexion(self._motion.flexion_target_deg)

    def _warn_on_flexion(self, tip_deg: float) -> None:
        """Mirror the firmware's overstretch ladder in the operator's log."""
        magnitude = abs(tip_deg)
        cfg = self._cfg.motion.flexion
        if magnitude >= cfg.major_deg:
            log.warning("flexion %.1f deg -- MAJOR overstretch risk", tip_deg)
        elif magnitude >= cfg.warning_deg:
            log.warning("flexion %.1f deg -- high overstretch risk", tip_deg)
        elif magnitude >= cfg.caution_deg:
            log.info("flexion %.1f deg -- entering precaution range", tip_deg)

    # -- emergency stop ----------------------------------------------------

    def _trigger_estop(self, reason: str) -> None:
        if self._estopped:
            return
        self._estopped = True
        self._stats.estops += 1
        if self.recorder is not None:
            self.recorder.mark(f"ESTOP: {reason}")
        self._fire_started_at = None      # never let a latch outlive an e-stop
        self._laser_action_at.clear()     # cooldown must never delay a stop
        log.critical("E-STOP: %s", reason)

        # Velocity mode can genuinely stop: the firmware is not blocked, so S
        # takes effect within a millisecond. Chunked mode can only stop issuing
        # new commands and let the current one finish.
        if self._velocity_mode:
            try:
                self._motion.stop()
                log.critical("motion stopped")
            except Exception as exc:  # noqa: BLE001 - must reach the laser stop
                log.critical("motion stop failed: %s", exc)

        if self._laser is not None:
            try:
                state = self._laser.stop()
                log.critical("laser stopped and relocked | %s", state.describe())
            except LaserError as exc:
                log.critical(
                    "LASER STOP FAILED: %s -- treat the machine as ARMED and "
                    "use the physical interlock",
                    exc,
                )

    def _clear_estop(self) -> None:
        self._estopped = False
        self._latched_firing = False
        self._fire_started_at = None
        self._edges.reset()
        log.warning("e-stop cleared; laser remains disarmed until Enable/Ready")

    # -- status and teardown -----------------------------------------------

    def status(self) -> dict:
        return {
            "estopped": self._estopped,
            "motion": self._motion.state,
            "flexion_target_deg": self._motion.flexion_target_deg,
            "laser": self._laser.state if self._laser else None,
            "stats": self._stats,
            "controller": self._source.name,
        }

    def shutdown(self) -> None:
        """Park everything. Safe to call more than once."""
        log.info("shutting down")
        if self._velocity_mode:
            try:
                self._motion.stop()
            except Exception as exc:  # noqa: BLE001
                log.error("could not stop motion during shutdown: %s", exc)

        if self._laser is not None:
            for attempt in (1, 2):
                try:
                    self._laser.stop()
                    log.info("laser relocked")
                    break
                except LaserError as exc:
                    if attempt == 1:
                        log.warning("relock attempt 1 failed (%s); retrying", exc)
                        continue
                    log.critical(
                        "COULD NOT RELOCK THE LASER: %s\n"
                        "Treat the machine as ARMED. Use the Dornier's own "
                        "standby and cut the relay supply.",
                        exc,
                    )
        self._running = False
