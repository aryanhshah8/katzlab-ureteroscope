"""Session recorder: joint positions, velocities and laser state to CSV.

The firmware already publishes everything needed -- position on all three axes,
both stepper velocities, and the laser's command state -- as TEL telemetry.
Nothing was writing it down. This does, at a fixed rate, with an ``event``
column so an operator action (arm, fire, home, e-stop) can be read against the
position it happened at.

One row per sample. Opens in Excel, pandas, MATLAB, anything.
"""

from __future__ import annotations

import csv
import logging
import time
from datetime import datetime
from pathlib import Path

log = logging.getLogger(__name__)

COLUMNS = [
    "t_s",              # seconds since the recording started
    "wall_clock",       # ISO timestamp, for lining up against video or notes
    "linear_mm",        # carriage position along the lead screw
    "rotation_deg",     # ureteroscope rotation, after the 2.5:1 reduction
    "flexion_deg",      # tip flexion angle
    "linear_mm_s",      # linear velocity
    "rotation_deg_s",   # rotation velocity
    "laser_enabled",
    "laser_ready",
    "laser_firing",
    "estopped",
    "event",            # blank except on an operator action
]


class SessionRecorder:
    """Appends a row per sample to a timestamped CSV."""

    def __init__(self, directory, *, rate_hz: float = 20.0) -> None:
        self._directory = Path(directory)
        self._period_s = 1.0 / max(0.1, rate_hz)
        self._handle = None
        self._writer = None
        self._started_at = 0.0
        self._last_row_at = 0.0
        self._pending_event = ""
        self.path = None
        self.rows = 0

    # -- lifecycle ---------------------------------------------------------

    def open(self):
        self._directory.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.path = self._directory / f"motion-{stamp}.csv"

        self._handle = self.path.open("w", newline="")
        self._writer = csv.writer(self._handle)
        self._writer.writerow(COLUMNS)
        self._started_at = time.monotonic()
        self._last_row_at = 0.0

        log.info("recording motion to %s", self.path)
        return self.path

    def close(self) -> None:
        if self._handle is None:
            return
        self._handle.flush()
        self._handle.close()
        self._handle = None
        self._writer = None
        if self.path is not None:
            log.info("recorded %d rows to %s", self.rows, self.path)

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    # -- recording ---------------------------------------------------------

    def mark(self, event: str) -> None:
        """Tag the next row with an operator action.

        Held rather than written immediately so the event lands on a row that
        also carries position -- an 'arm' with no coordinates beside it is not
        much use when reviewing a run.
        """
        self._pending_event = event

    def sample(self, motion_state, laser_state, *, estopped: bool = False,
               force: bool = False) -> None:
        """Write a row if the sample period has elapsed, or if forced.

        A pending event always forces a row: operator actions are the points you
        go looking for afterwards, and dropping one to stay on a fixed cadence
        would be the wrong trade.
        """
        if self._writer is None:
            return

        now = time.monotonic()
        elapsed = now - self._started_at
        due = (elapsed - self._last_row_at) >= self._period_s

        if not (due or force or self._pending_event):
            return

        self._last_row_at = elapsed
        event, self._pending_event = self._pending_event, ""

        self._writer.writerow([
            f"{elapsed:.3f}",
            datetime.now().isoformat(timespec="milliseconds"),
            f"{getattr(motion_state, 'linear_mm', 0.0):.4f}",
            f"{getattr(motion_state, 'rotation_map_deg', 0.0):.3f}",
            f"{getattr(motion_state, 'flexion_tip_deg', 0.0):.3f}",
            f"{getattr(motion_state, 'linear_velocity_mm_s', 0.0):.4f}",
            f"{getattr(motion_state, 'rotation_velocity_deg_s', 0.0):.3f}",
            int(bool(getattr(laser_state, "enabled", False))),
            int(bool(getattr(laser_state, "ready", False))),
            int(bool(getattr(laser_state, "firing", False))),
            int(bool(estopped)),
            event,
        ])
        self.rows += 1

        # Flush on events so a crash cannot lose the interesting rows.
        if event and self._handle is not None:
            self._handle.flush()
