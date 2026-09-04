"""Driver for the 3-DOF motion sketch (``3DOF_MovementF.ino``).

Protocol, as the sketch actually implements it::

    <linear_mm_relative> <rotation_deg_relative> <flexion_deg_absolute>\n
    home | h

The sketch prints a prompt, blocks on ``while (Serial.available() == 0) {}``,
then runs the whole move to completion before prompting again. That has two
consequences this driver is built around:

1. Every command is a blocking round trip. The prompt banner is the only
   reliable "ready" signal, so we synchronise on it.
2. A move cannot be interrupted once started. The mitigation is to keep each
   commanded move short -- see ``motion.*.max_chunk_*`` in system.yaml -- so
   that an e-stop is never waiting on more than one chunk.

The sketch rejects an entire three-axis command if linear or flexion would leave
range, so this driver pre-clamps against the same limits. A rejection is treated
as a bug in the clamping, not as a normal outcome.
"""

from __future__ import annotations

import logging
import re
import time
from dataclasses import dataclass, replace

from ..config import MotionConfig
from ..transport import Link, LinkTimeout

log = logging.getLogger(__name__)

# The sketch's per-command prompt. Printed once per loop iteration, including
# after a rejected command, so it is always the resynchronisation point.
PROMPT = r"POSITIONING: Linear = RELATIVE"
COMPLETE = r"3-DOF Movement Complete"

RE_LINEAR_FINAL = re.compile(r"Final Linear Absolute Position:\s*(-?\d+\.?\d*)")
RE_ROTATION_FINAL = re.compile(r"Final Rotation Map Position:\s*(-?\d+\.?\d*)")
RE_FLEXION_FINAL = re.compile(
    r"Final Flexion Step, Motor Deg, Tip Deg:\s*(-?\d+),\s*(-?\d+\.?\d*),\s*(-?\d+\.?\d*)"
)

ERROR_MARKERS = ("Wrong Format", "Linear Range Error", "Flexion Range Error")


class MotionError(RuntimeError):
    """The firmware rejected a motion command."""


@dataclass(frozen=True)
class MotionState:
    """Last known position, as reported by the firmware itself."""

    linear_mm: float = 0.0
    rotation_map_deg: float = 0.0
    flexion_tip_deg: float = 0.0
    flexion_step: int = -1
    flexion_motor_deg: float = 0.0

    def describe(self) -> str:
        return (
            f"linear {self.linear_mm:7.3f} mm | "
            f"rotation {self.rotation_map_deg:7.2f} deg | "
            f"flexion {self.flexion_tip_deg:7.2f} deg"
        )


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class MotionDriver:
    """Speaks the 3-DOF sketch's text protocol."""

    def __init__(self, link: Link | None, cfg: MotionConfig, *, dry_run: bool = False) -> None:
        self._link = link
        self._cfg = cfg
        self._dry_run = dry_run
        self._state = MotionState()
        self._synced = False
        # Absolute flexion target held on the Python side. The stick integrates
        # into this, so releasing the stick holds the angle instead of springing
        # the tip back toward zero.
        self._flexion_target_deg = 0.0
        # Last value actually sent to the servo. Re-commanding an absolute
        # target 20x a second interrupts the servo mid-move every time, which
        # is what makes flexion feel erratic. Only re-command once the intent
        # has drifted past a deadband, so the servo gets fewer, longer, smoother
        # moves it can actually finish.
        self._last_sent_flexion_deg = 0.0

    # -- state -------------------------------------------------------------

    @property
    def state(self) -> MotionState:
        return self._state

    @property
    def flexion_target_deg(self) -> float:
        return self._flexion_target_deg

    def set_flexion_target(self, deg: float) -> float:
        self._flexion_target_deg = _clamp(
            deg, self._cfg.flexion.min_deg, self._cfg.flexion.max_deg
        )
        return self._flexion_target_deg

    # -- synchronisation ---------------------------------------------------

    def sync(self, timeout_s: float = 15.0) -> None:  # noqa: D401
        """Wait for the sketch's prompt so we know it is listening.

        The Teensy reboots on port open and prints a long diagnostics block
        first, so this is generous with its timeout.

        Call this once, after opening the link. It waits passively for output
        the firmware sends unprompted, so calling it later -- once that output
        has been consumed -- would simply block until it timed out. Every
        command after this point carries its own response.
        """
        if self._dry_run:
            self._synced = True
            log.info("motion DRY RUN -- no serial link, nothing will move")
            return

        line, gathered = self._link.wait_for([PROMPT], timeout_s=timeout_s)
        self._absorb_telemetry(gathered + [line])
        self._synced = True
        log.info("motion firmware ready | %s", self._state.describe())

    # -- commands ----------------------------------------------------------

    def move(
        self,
        linear_mm: float = 0.0,
        rotation_deg: float = 0.0,
        flexion_abs_deg: float | None = None,
        *,
        timeout_s: float = 30.0,
    ) -> MotionState:
        """Send one relative linear / relative rotation / absolute flexion move.

        ``linear_mm`` and ``rotation_deg`` are relative to the current position;
        ``flexion_abs_deg`` is an absolute tip angle. Values are clamped to the
        configured envelope before being sent.
        """
        linear_mm = self._clamp_linear(linear_mm)
        rotation_deg = self._clamp_rotation(rotation_deg)

        if flexion_abs_deg is None:
            flexion_abs_deg = self._flexion_target_deg
        flexion_abs_deg = self.set_flexion_target(flexion_abs_deg)

        # Below the deadband, re-send the previous value. The firmware treats an
        # unchanged flexion target as already satisfied and does not wait on the
        # servo at all, so this costs nothing and keeps the servo moving cleanly.
        if (
            flexion_abs_deg != 0.0
            and abs(flexion_abs_deg - self._last_sent_flexion_deg)
            < self._cfg.flexion.command_deadband_deg
        ):
            flexion_to_send = self._last_sent_flexion_deg
        else:
            flexion_to_send = flexion_abs_deg
            self._last_sent_flexion_deg = flexion_abs_deg

        command = f"{linear_mm:.4f} {rotation_deg:.4f} {flexion_to_send:.3f}"
        return self._exchange(command, timeout_s=timeout_s)

    def home(self, timeout_s: float = 180.0, should_abort=None) -> MotionState:
        """Return all three axes to the software home state, in chunks.

        The firmware's own `home` command is ONE blocking move across the whole
        travel -- up to two minutes during which nothing, including an e-stop,
        can interrupt it. That is not acceptable for a control this easy to hit
        by accident, so homing is driven from here as a series of ordinary
        chunked moves instead.

        ``should_abort`` is polled between chunks; return True from it to stop.
        """
        log.info("homing all axes (interruptible)")

        # Flexion is absolute: one command retargets it, no traverse needed.
        self._flexion_target_deg = 0.0

        deadline = time.monotonic() + timeout_s
        linear_chunk = self._cfg.linear.max_chunk_mm
        rotation_chunk = self._cfg.rotation.max_chunk_deg

        while True:
            if should_abort is not None and should_abort():
                log.warning("home aborted at %s", self._state.describe())
                return self._state

            if time.monotonic() > deadline:
                log.error("home timed out at %s", self._state.describe())
                return self._state

            linear_error = -self._state.linear_mm
            rotation_error = -self._state.rotation_map_deg

            linear_done = abs(linear_error) <= 0.05
            rotation_done = abs(rotation_error) <= 0.5
            flexion_done = abs(self._last_sent_flexion_deg) <= 0.01

            if linear_done and rotation_done and flexion_done:
                log.info("home complete | %s", self._state.describe())
                return self._state

            self.move(
                linear_mm=_clamp(linear_error, -linear_chunk, linear_chunk),
                rotation_deg=_clamp(rotation_error, -rotation_chunk, rotation_chunk),
                flexion_abs_deg=0.0,
            )

    # -- internals ---------------------------------------------------------

    def _exchange(self, command: str, *, timeout_s: float) -> MotionState:
        if self._dry_run:
            return self._simulate(command)

        # Discard anything still queued from an earlier command. The firmware
        # reprints its prompt after every command -- including laser ones on a
        # shared link -- and a stale prompt would satisfy the wait below
        # instantly, pairing this command with the previous one's telemetry.
        self._link.drain()
        self._link.write_line(command)
        try:
            line, gathered = self._link.wait_for([PROMPT], timeout_s=timeout_s)
        except LinkTimeout as exc:
            self._synced = False
            raise MotionError(
                f"no prompt after {timeout_s:.0f}s for command {command!r}. "
                "The board may still be executing a move, or the sketch is not "
                "the one this driver expects."
            ) from exc

        lines = gathered + [line]
        self._absorb_telemetry(lines)
        self._synced = True

        for marker in ERROR_MARKERS:
            if any(marker in text for text in lines):
                raise MotionError(
                    f"firmware rejected {command!r} with {marker!r}. This means "
                    "Python's limit clamping disagrees with the firmware's -- "
                    "check motion.* in system.yaml against the .ino constants."
                )

        return self._state

    def _simulate(self, command: str) -> MotionState:
        """Dead-reckon the move instead of sending it.

        Only for --dry-run. Real runs never guess at position: they read it back
        out of the firmware's own summary lines.
        """
        log.info("DRY RUN motion -> %s", command)
        if command in ("home", "h"):
            self._state = MotionState()
            return self._state

        linear, rotation, flexion = (float(v) for v in command.split())
        self._state = replace(
            self._state,
            linear_mm=self._state.linear_mm + linear,
            rotation_map_deg=self._state.rotation_map_deg + rotation,
            flexion_tip_deg=flexion,
        )
        return self._state

    def _absorb_telemetry(self, lines: list[str]) -> None:
        """Pull position out of the firmware's own prints.

        Position always comes from the board rather than from dead reckoning in
        Python, so a skipped step or a rejected command cannot silently
        desynchronise the two.
        """
        state = self._state
        for text in lines:
            if match := RE_LINEAR_FINAL.search(text):
                state = replace(state, linear_mm=float(match.group(1)))
            if match := RE_ROTATION_FINAL.search(text):
                state = replace(state, rotation_map_deg=float(match.group(1)))
            if match := RE_FLEXION_FINAL.search(text):
                state = replace(
                    state,
                    flexion_step=int(match.group(1)),
                    flexion_motor_deg=float(match.group(2)),
                    flexion_tip_deg=float(match.group(3)),
                )
        self._state = state

    def _clamp_linear(self, delta_mm: float) -> float:
        cfg = self._cfg.linear
        delta_mm = _clamp(delta_mm, -cfg.max_chunk_mm, cfg.max_chunk_mm)
        target = self._state.linear_mm + delta_mm
        clamped_target = _clamp(target, cfg.min_mm, cfg.max_mm)
        return clamped_target - self._state.linear_mm

    def _clamp_rotation(self, delta_deg: float) -> float:
        # Rotation has no software end stop in the firmware; only the per-command
        # chunk size is bounded, to keep each blocking move short.
        cfg = self._cfg.rotation
        return _clamp(delta_deg, -cfg.max_chunk_deg, cfg.max_chunk_deg)
