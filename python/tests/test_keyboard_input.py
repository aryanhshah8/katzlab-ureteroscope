"""Keyboard input source: key parsing and the held/decay model.

None of this touches a real terminal. `_parse_buffer` and the hold-timeout
logic in `poll()` are pure functions of (bytes received, monotonic time), which
is exactly what makes them testable without a tty -- feed synthetic bytes
straight into `_drain`/`_parse_buffer` instead of going through `open()`.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import pytest

from katzlab.input.keyboard import HOLD_TIMEOUT_S, KeyboardInputSource


def feed(src: KeyboardInputSource, data: bytes, at: float) -> None:
    """Inject bytes as if they had just arrived on stdin, at time `at`."""
    src._buffer.extend(data)
    src._parse_buffer(at)


@pytest.fixture
def src():
    return KeyboardInputSource()


# --------------------------------------------------------------------------
# single-byte keys
# --------------------------------------------------------------------------


def test_d_drives_linear_positive(src, monkeypatch):
    monkeypatch.setattr("time.monotonic", lambda: 0.0)
    feed(src, b"d", at=0.0)
    assert src.poll().axis("linear") == pytest.approx(1.0)


def test_a_drives_linear_negative(src, monkeypatch):
    monkeypatch.setattr("time.monotonic", lambda: 0.0)
    feed(src, b"a", at=0.0)
    assert src.poll().axis("linear") == pytest.approx(-1.0)


def test_opposing_keys_cancel_when_held_together(src, monkeypatch):
    monkeypatch.setattr("time.monotonic", lambda: 10.0)
    feed(src, b"a", at=10.0)
    feed(src, b"d", at=10.0)
    frame = src.poll()
    assert frame.axis("linear") == pytest.approx(0.0)


def test_keys_are_case_insensitive(src, monkeypatch):
    monkeypatch.setattr("time.monotonic", lambda: 5.0)
    feed(src, b"D", at=5.0)
    frame = src.poll()
    assert frame.axis("linear") == pytest.approx(1.0)

    feed(src, b"F", at=5.0)
    frame = src.poll()
    assert frame.button("fire") is True


def test_all_six_buttons_are_reachable(src, monkeypatch):
    now = [0.0]
    monkeypatch.setattr("time.monotonic", lambda: now[0])

    mapping = {
        b"s": "standby",
        b"h": "home",
        b"x": "estop",
        b"p": "pause",
        b"f": "fire",
        b"r": "enable_ready",
    }
    for key, button in mapping.items():
        feed(src, key, at=now[0])
        frame = src.poll()
        assert frame.button(button) is True, f"{key!r} did not drive {button!r}"
        # Let it decay before testing the next one so buttons cannot leak
        # into each other via stale timestamps.
        now[0] += HOLD_TIMEOUT_S + 0.5


def test_unmapped_keys_are_silently_ignored(src, monkeypatch):
    monkeypatch.setattr("time.monotonic", lambda: 0.0)
    feed(src, b"qzj9", at=0.0)
    frame = src.poll()
    assert not frame.any_axis_active
    assert not any(frame.buttons.values())


# --------------------------------------------------------------------------
# arrow keys (rotation / flexion)
# --------------------------------------------------------------------------


def test_up_and_down_drive_rotation(src, monkeypatch):
    monkeypatch.setattr("time.monotonic", lambda: 0.0)
    feed(src, b"\x1b[A", at=0.0)  # Up
    assert src.poll().axis("rotation") == pytest.approx(1.0)

    feed(src, b"\x1b[B", at=0.0)  # Down
    assert src.poll().axis("rotation") == pytest.approx(0.0)  # both held: cancel


def test_left_and_right_drive_flexion(src, monkeypatch):
    monkeypatch.setattr("time.monotonic", lambda: 0.0)
    feed(src, b"\x1b[C", at=0.0)  # Right
    assert src.poll().axis("flexion") == pytest.approx(1.0)


def test_arrow_sequence_split_across_two_reads_is_not_misparsed(src, monkeypatch):
    """A slow link (or an unlucky read boundary) can deliver an escape
    sequence one or two bytes at a time. The parser must wait for the rest
    rather than treating a lone ESC as something else."""
    monkeypatch.setattr("time.monotonic", lambda: 0.0)
    feed(src, b"\x1b", at=0.0)
    feed(src, b"[A", at=0.0)
    assert src.poll().axis("rotation") == pytest.approx(1.0)


def test_a_lone_escape_key_does_not_crash_or_hang(src, monkeypatch):
    monkeypatch.setattr("time.monotonic", lambda: 0.0)
    feed(src, b"\x1b", at=0.0)
    # No follow-up bytes ever arrive (a real Escape key press). Nothing should
    # be interpreted as held, and the buffer should not grow without bound.
    frame = src.poll()
    assert not frame.any_axis_active
    assert len(src._buffer) <= 1


def test_unrecognised_escape_sequence_is_consumed_not_stuck(src, monkeypatch):
    monkeypatch.setattr("time.monotonic", lambda: 0.0)
    feed(src, b"\x1b[Z", at=0.0)  # not one of the four arrow letters
    src.poll()
    assert len(src._buffer) == 0


# --------------------------------------------------------------------------
# the hold/decay model
# --------------------------------------------------------------------------


def test_a_single_press_decays_to_released_after_the_timeout(src, monkeypatch):
    now = [0.0]
    monkeypatch.setattr("time.monotonic", lambda: now[0])

    feed(src, b"f", at=0.0)
    assert src.poll().button("fire") is True

    now[0] = HOLD_TIMEOUT_S + 0.1
    assert src.poll().button("fire") is False


def test_repeated_bytes_within_the_timeout_read_as_one_continuous_hold(src, monkeypatch):
    """This is what OS key-repeat produces while a key is physically down --
    a stream of the same byte, closer together than HOLD_TIMEOUT_S."""
    now = [0.0]
    monkeypatch.setattr("time.monotonic", lambda: now[0])

    for _ in range(5):
        feed(src, b"f", at=now[0])
        assert src.poll().button("fire") is True
        now[0] += HOLD_TIMEOUT_S * 0.5   # repeats faster than the timeout

    now[0] += HOLD_TIMEOUT_S + 0.1
    assert src.poll().button("fire") is False


# --------------------------------------------------------------------------
# lifecycle
# --------------------------------------------------------------------------


def test_open_refuses_a_non_interactive_stdin(monkeypatch):
    monkeypatch.setattr("sys.stdin.isatty", lambda: False)
    src = KeyboardInputSource()
    from katzlab.input.base import InputUnavailable

    with pytest.raises(InputUnavailable):
        src.open()


def test_name_identifies_the_source():
    assert "keyboard" in KeyboardInputSource().name.lower()


def test_construction_accepts_and_ignores_gamepad_only_kwargs():
    """Must drop into the same create_input_source(...) call sites used for
    the gamepad without those call sites needing to change."""
    src = KeyboardInputSource(
        controller=None,
        deadzone=0.06,
        expo=0.35,
        axis_expo={"flexion": 0.8},
        axis_deadzone={"flexion": 0.02},
        paired_axes=[{"axes": ["rotation", "flexion"]}],
    )
    assert src.name
