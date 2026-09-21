"""Unit tests for the ROS-facing InputSource and the SI->axis conversion.

No ROS install needed: Ros2InputSource only imports from katzlab.input.base,
so this runs anywhere the katzlab package itself does (see conftest.py for
the sys.path setup). bridge_node.py does import rclpy at module scope, so
its tests are skipped if rclpy isn't importable -- that half only runs where
ROS 2 is actually installed.
"""

from __future__ import annotations

import time

import pytest

from katzlab_bridge.ros_input_source import CMD_VELOCITY_STALE_S, HOME_HOLD_WINDOW_S, Ros2InputSource


def test_open_sets_last_cmd_so_a_fresh_source_is_not_immediately_stale():
    source = Ros2InputSource()
    source.open()
    frame = source.poll()
    assert frame.connected is True
    assert frame.axis("linear") == 0.0


def test_set_axes_is_reflected_in_the_next_poll():
    source = Ros2InputSource()
    source.open()
    source.set_axes(linear=0.5, rotation=-0.25, flexion=1.0)

    frame = source.poll()
    assert frame.axis("linear") == pytest.approx(0.5)
    assert frame.axis("rotation") == pytest.approx(-0.25)
    assert frame.axis("flexion") == pytest.approx(1.0)


def test_set_axes_clamps_to_unit_range():
    source = Ros2InputSource()
    source.open()
    source.set_axes(linear=5.0, rotation=-5.0, flexion=0.0)

    frame = source.poll()
    assert frame.axis("linear") == 1.0
    assert frame.axis("rotation") == -1.0


def test_stale_cmd_velocity_zeroes_axes_but_stays_connected():
    """A dropped cmd_velocity publisher must fail toward zero speed, not

    toward looking like a disconnected controller -- ControlLoop's own
    watchdog keys off ``frame.connected``, which the ROS node itself stays
    alive to report True regardless of whether commands keep arriving.
    """
    source = Ros2InputSource()
    source.open()
    source.set_axes(linear=1.0, rotation=1.0, flexion=1.0)

    # Force staleness without a real sleep.
    source._last_cmd_at = time.monotonic() - (CMD_VELOCITY_STALE_S + 0.1)

    frame = source.poll()
    assert frame.connected is True
    assert frame.axis("linear") == 0.0
    assert frame.axis("rotation") == 0.0
    assert frame.axis("flexion") == 0.0


def test_request_home_holds_the_home_button_for_the_window():
    source = Ros2InputSource()
    source.open()
    source.request_home()

    frame = source.poll()
    assert frame.button("home") is True

    # Still held well within the window.
    source._home_until = time.monotonic() + (HOME_HOLD_WINDOW_S - 0.05)
    assert source.poll().button("home") is True

    # Expired.
    source._home_until = time.monotonic() - 0.01
    assert source.poll().button("home") is False


def test_request_estop_is_a_one_shot_edge():
    source = Ros2InputSource()
    source.open()
    source.request_estop()

    first = source.poll()
    assert first.button("estop") is True

    second = source.poll()
    assert second.button("estop") is False


def test_axis_conversion_matches_bridge_node_to_axis():
    pytest.importorskip("rclpy", reason="only meaningful where ROS 2 is installed")
    from katzlab_bridge.bridge_node import _to_axis

    assert _to_axis(0.0, 2.5) == 0.0
    assert _to_axis(2.5, 2.5) == pytest.approx(1.0)
    assert _to_axis(-2.5, 2.5) == pytest.approx(-1.0)
    assert _to_axis(5.0, 2.5) == 1.0        # clamped
    assert _to_axis(1.0, 0.0) == 0.0        # no divide-by-zero
