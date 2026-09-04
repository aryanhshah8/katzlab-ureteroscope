"""Driver for the velocity-mode motion firmware (``Motion_Velocity_Server.ino``).

The difference from :mod:`katzlab.drivers.motion` is the whole point of it: that
driver sends a *distance* and blocks until the firmware has moved it, which
means a full stop between every command. This one sends a *velocity* and returns
immediately. The firmware ramps toward it and keeps stepping.

Consequences that matter at the bench:

* Motion is continuous. No stop between commands, so no stutter, and the
  firmware's acceleration ramp means the motor is never asked to jump to speed
  from standstill.
* Speed follows the stick continuously, because the stick position *is* the
  commanded velocity. There is no integration step to get wrong.
* Stopping is immediate. Nothing blocks, so ``S`` takes effect within a
  millisecond instead of waiting for a move to finish.

Position arrives unprompted as ``TEL`` telemetry rather than being parsed out of
a command's reply.
"""

from __future__ import annotations

import logging
import re
import time
from dataclasses import dataclass

from ..config import MotionConfig
from ..transport import Link, LinkTimeout

log = logging.getLogger(__name__)

RE_TEL = re.compile(
    r"TEL\s+L(-?\d+\.?\d*)\s+R(-?\d+\.?\d*)\s+F(-?\d+\.?\d*)"
    r"\s+VL(-?\d+\.?\d*)\s+VR(-?\d+\.?\d*)"
    r"\s+E([01])\s+Y([01])\s+G([01])\s+H([01])"
)

BANNER = r"MOTION VELOCITY SERVER"


class MotionError(RuntimeError):
    """The velocity firmware did not respond as expected."""


@dataclass(frozen=True)
class MotionState:
    linear_mm: float = 0.0
    rotation_map_deg: float = 0.0
    flexion_tip_deg: float = 0.0
    linear_velocity_mm_s: float = 0.0
    rotation_velocity_deg_s: float = 0.0
    homing: bool = False

    # Kept so callers written against the chunked driver still work.
    flexion_step: int = -1
    flexion_motor_deg: float = 0.0

    def describe(self) -> str:
        return (
            f"linear {self.linear_mm:7.3f} mm ({self.linear_velocity_mm_s:+5.2f} mm/s) | "
            f"rotation {self.rotation_map_deg:7.2f} deg | "
            f"flexion {self.flexion_tip_deg:7.2f} deg"
            + ("  [homing]" if self.homing else "")
        )


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class VelocityMotionDriver:
    """Speaks the velocity server's protocol."""

    #: Lets ControlLoop tell the two motion drivers apart.
    is_velocity_mode = True

    def __init__(self, link: Link, cfg: MotionConfig) -> None:
        self._link = link
        self._cfg = cfg
        self._state = MotionState()
        self._flexion_target_deg = 0.0
        self._last_sent = (None, None, None)

    # -- state -------------------------------------------------------------

    @property
    def state(self) -> MotionState:
        self._absorb()
        return self._state

    @property
    def flexion_target_deg(self) -> float:
        return self._flexion_target_deg

    def set_flexion_target(self, deg: float) -> float:
        self._flexion_target_deg = _clamp(
            deg, self._cfg.flexion.min_deg, self._cfg.flexion.max_deg
        )
        return self._flexion_target_deg

    # -- lifecycle ---------------------------------------------------------

    def sync(self, timeout_s: float = 15.0) -> None:
        """Confirm the velocity firmware is what is actually running.

        A wrong sketch here is not a cosmetic problem -- the chunked firmware
        would read "L2.5" as a malformed move -- so this refuses rather than
        guesses.
        """
        try:
            self._link.wait_for([BANNER], timeout_s=timeout_s)
            log.info("velocity firmware ready")
        except LinkTimeout:
            self._link.drain()
            self._link.write_line("?")
            try:
                self._link.wait_for([r"TEL\s+L"], timeout_s=5.0)
            except LinkTimeout as exc:
                raise MotionError(
                    "no response from the velocity firmware. Is "
                    "Motion_Velocity_Server flashed? The chunked "
                    "Combined_Motion_Laser sketch does not speak this protocol."
                ) from exc
            log.info("velocity firmware ready (via status query)")

        self.set_telemetry_period(100)
        self.set_flexion_deadband(self._cfg.flexion.firmware_deadband_deg)
        self.stop()

    def set_telemetry_period(self, period_ms: int) -> None:
        self._link.write_line(f"T{period_ms}")

    def set_flexion_deadband(self, degrees: float) -> None:
        """How far the target must drift before the servo is re-commanded.

        Runtime-set rather than compiled in, because the right value depends on
        the flexion rate -- the servo command interval is deadband / rate, so
        slowing the axis down without shrinking this turns smooth motion into
        discrete hops.
        """
        self._link.write_line(f"D{degrees:.3f}")

    # -- commands ----------------------------------------------------------

    def set_velocity(
        self,
        linear_mm_s: float = 0.0,
        rotation_deg_s: float = 0.0,
        flexion_abs_deg: float | None = None,
    ) -> MotionState:
        """Set target velocities. Returns immediately -- nothing blocks."""
        linear_mm_s = _clamp(
            linear_mm_s,
            -self._cfg.linear.max_rate_mm_s,
            self._cfg.linear.max_rate_mm_s,
        )
        rotation_deg_s = _clamp(
            rotation_deg_s,
            -self._cfg.rotation.max_rate_deg_s,
            self._cfg.rotation.max_rate_deg_s,
        )

        if flexion_abs_deg is not None:
            self.set_flexion_target(flexion_abs_deg)

        _, _, last_f = self._last_sent

        # Velocity is sent EVERY call, even when unchanged. The firmware's
        # watchdog treats a quiet link as a dead host and ramps to a stop, so
        # suppressing "redundant" writes made a held stick stall after the
        # timeout -- the command had not changed, so nothing was sent, so the
        # firmware concluded the host was gone. Two velocity lines per tick is
        # ~1.2 kB/s at 60 Hz against 11.5 kB/s available; the bandwidth was
        # never worth the failure mode.
        self._link.write_line(f"L{linear_mm_s:.3f}")
        self._link.write_line(f"R{rotation_deg_s:.2f}")

        # Flexion is the exception. It is an absolute servo target, and
        # re-commanding it restarts the servo's internal acceleration ramp,
        # which is what made flexion stutter. Only send it when it has actually
        # moved -- the velocity lines above already feed the watchdog.
        if last_f is None or abs(self._flexion_target_deg - last_f) > 0.05:
            self._link.write_line(f"F{self._flexion_target_deg:.2f}")
            last_f = self._flexion_target_deg

        self._last_sent = (linear_mm_s, rotation_deg_s, last_f)
        return self.state

    def heartbeat(self) -> None:
        """Keep the firmware watchdog fed when nothing has changed."""
        self._link.write_line("?")

    def stop(self) -> MotionState:
        self._link.write_line("S")
        self._last_sent = (0.0, 0.0, self._last_sent[2])
        return self.state

    def zero(self) -> MotionState:
        """Treat the current position as 0 mm / 0 deg."""
        self._link.write_line("Z")
        return self.state

    def home(self, timeout_s: float = 180.0, should_abort=None) -> MotionState:
        """Ask the firmware to home, and watch until it reports done.

        Homing runs inside the firmware as a velocity profile, so it is smooth
        and can be cancelled at any instant -- ``should_abort`` sends ``S``,
        which takes effect immediately rather than after the current move.
        """
        log.info("homing (velocity mode, abortable)")
        self._link.write_line("H")
        self._flexion_target_deg = 0.0
        self._last_sent = (None, None, 0.0)

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if should_abort is not None and should_abort():
                self.stop()
                log.warning("home aborted at %s", self._state.describe())
                return self._state

            state = self.state
            if not state.homing and abs(state.linear_mm) <= 0.1:
                log.info("home complete | %s", state.describe())
                return state
            time.sleep(0.02)

        self.stop()
        log.error("home timed out at %s", self._state.describe())
        return self._state

    # -- telemetry ---------------------------------------------------------

    def _absorb(self) -> None:
        # Take ONLY telemetry. This runs on every control tick, and on a
        # single-board rig the laser is on the same port -- an unfiltered drain
        # here swallows the laser's command replies, which then time out and
        # leave its state mirror wrong. That is what made the laser behave
        # erratically: the commands were fine, their answers were being eaten.
        for line in self._link.drain(lambda l: "TEL" in l):
            if match := RE_TEL.search(line):
                self._state = MotionState(
                    linear_mm=float(match.group(1)),
                    rotation_map_deg=float(match.group(2)),
                    flexion_tip_deg=float(match.group(3)),
                    linear_velocity_mm_s=float(match.group(4)),
                    rotation_velocity_deg_s=float(match.group(5)),
                    homing=match.group(9) == "1",
                )
