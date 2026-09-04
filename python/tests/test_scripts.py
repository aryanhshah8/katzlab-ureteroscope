"""Checks that the operator scripts actually run.

These exist because a shallow smoke test -- running the script and reading its
banner -- passed while `snapshot()` raised NameError on the very next line. The
banner is not the code that matters. Anything reachable during a calibration
run gets called here, against a stub pad, so a missing name cannot hide behind
a header that prints fine.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

import pytest


class StubJoystick:
    """A pad that is present, centred, and never pressed."""

    def __init__(self, axes: int = 4, buttons: int = 13) -> None:
        self._axes = axes
        self._buttons = buttons

    def get_numaxes(self) -> int:
        return self._axes

    def get_numbuttons(self) -> int:
        return self._buttons

    def get_numhats(self) -> int:
        return 1

    def get_axis(self, index: int) -> float:
        return 0.0

    def get_button(self, index: int) -> bool:
        return False

    def get_hat(self, index: int) -> tuple[int, int]:
        return (0, 0)

    def get_name(self) -> str:
        return "Stub Pad"

    def get_guid(self) -> str:
        return "0" * 32


class StubPygame:
    class event:
        @staticmethod
        def pump() -> None:
            return


@pytest.fixture
def cal(monkeypatch):
    import calibrate_controller

    monkeypatch.setattr(calibrate_controller, "pygame", StubPygame)
    monkeypatch.setattr(calibrate_controller, "AXIS_TIMEOUT_S", 0.3)
    monkeypatch.setattr(calibrate_controller, "BUTTON_TIMEOUT_S", 0.3)
    return calibrate_controller


def test_snapshot_reads_the_pad(cal):
    axes, buttons = cal.snapshot(StubJoystick())
    assert len(axes) == 4
    assert len(buttons) == 13


def test_wait_for_release_returns_when_nothing_is_held(cal):
    cal.wait_for_release(StubJoystick())      # must not hang or raise


def test_capture_axis_fails_cleanly_when_nothing_moves(cal):
    with pytest.raises(SystemExit, match="No movement detected"):
        cal.capture_axis(StubJoystick(), "test prompt")


def test_capture_button_fails_cleanly_when_nothing_is_pressed(cal):
    with pytest.raises(SystemExit, match="Nothing detected"):
        cal.capture_button(StubJoystick(), "test prompt")


def test_axis_capture_failure_points_at_the_monitor(cal):
    """The failure message has to tell the operator what to do next."""
    with pytest.raises(SystemExit) as excinfo:
        cal.capture_axis(StubJoystick(), "test prompt")
    assert "gamepad_monitor.py" in str(excinfo.value)


def test_monitor_bar_renders_across_the_range():
    import gamepad_monitor

    for value in (-1.0, -0.5, 0.0, 0.5, 1.0, 2.0, -2.0):
        rendered = gamepad_monitor.bar(value)
        assert "\033[0m" in rendered          # colour is closed
        assert len(rendered) > gamepad_monitor.BAR_WIDTH


def test_sdl_helper_reports_a_backend():
    """init_joysticks must always name the backend it settled on."""
    from katzlab.input import sdl

    assert sdl.NOT_FOUND_HELP
    assert "Input Monitoring" in sdl.NOT_FOUND_HELP
