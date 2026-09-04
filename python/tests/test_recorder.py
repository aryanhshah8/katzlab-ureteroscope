"""Session CSV recording."""

from __future__ import annotations

import csv
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import pytest

from katzlab.drivers.laser import LaserState
from katzlab.drivers.motion_velocity import MotionState
from katzlab.recorder import COLUMNS, SessionRecorder


@pytest.fixture
def recorder(tmp_path):
    rec = SessionRecorder(tmp_path, rate_hz=1000.0)
    rec.open()
    yield rec
    rec.close()


def _read(path):
    with open(path, newline="") as handle:
        return list(csv.DictReader(handle))


def test_header_and_a_row_land_in_the_file(recorder):
    recorder.sample(MotionState(linear_mm=12.5, rotation_map_deg=-30.25,
                                flexion_tip_deg=45.5), LaserState(), force=True)
    recorder.close()

    rows = _read(recorder.path)
    assert list(rows[0].keys()) == COLUMNS
    assert float(rows[0]["linear_mm"]) == pytest.approx(12.5)
    assert float(rows[0]["rotation_deg"]) == pytest.approx(-30.25)
    assert float(rows[0]["flexion_deg"]) == pytest.approx(45.5)


def test_laser_state_is_recorded_alongside_position(recorder):
    recorder.sample(MotionState(linear_mm=3.0),
                    LaserState(enabled=True, ready=True, firing=True), force=True)
    recorder.close()

    row = _read(recorder.path)[0]
    assert row["laser_enabled"] == "1"
    assert row["laser_ready"] == "1"
    assert row["laser_firing"] == "1"


def test_an_event_forces_a_row_and_carries_position(recorder):
    """Operator actions are the rows you go looking for. They must never be
    dropped to keep a fixed sample cadence, and must carry coordinates."""
    slow = SessionRecorder(recorder.path.parent, rate_hz=0.001)   # ~never due
    slow.open()
    slow.mark("laser_fire")
    slow.sample(MotionState(linear_mm=7.25), LaserState(firing=True))
    slow.close()

    rows = _read(slow.path)
    assert len(rows) == 1, "event row was dropped by the sample rate"
    assert rows[0]["event"] == "laser_fire"
    assert float(rows[0]["linear_mm"]) == pytest.approx(7.25)


def test_rate_limiting_holds_between_events(tmp_path):
    rec = SessionRecorder(tmp_path, rate_hz=5.0)     # one row per 200 ms
    rec.open()
    for _ in range(50):
        rec.sample(MotionState(), LaserState())
        time.sleep(0.005)
    rec.close()

    rows = _read(rec.path)
    assert 1 <= len(rows) <= 4, f"expected ~2 rows in 250 ms at 5 Hz, got {len(rows)}"


def test_events_are_flushed_immediately(recorder):
    """A crash must not lose the interesting rows."""
    recorder.mark("ESTOP: X pressed")
    recorder.sample(MotionState(linear_mm=1.0), LaserState())

    rows = _read(recorder.path)          # read WITHOUT closing
    assert rows and rows[-1]["event"] == "ESTOP: X pressed"


def test_sampling_after_close_is_harmless(recorder):
    recorder.close()
    recorder.sample(MotionState(), LaserState(), force=True)   # must not raise


def test_missing_fields_default_rather_than_crash(recorder):
    """The chunked driver's MotionState has no velocity fields."""
    class Partial:
        linear_mm = 5.0

    recorder.sample(Partial(), None, force=True)
    recorder.close()

    row = _read(recorder.path)[0]
    assert float(row["linear_mm"]) == pytest.approx(5.0)
    assert float(row["rotation_deg_s"]) == pytest.approx(0.0)
    assert row["laser_firing"] == "0"
