"""Pluggable controller input.

The controller may end up plugged into the MacBook or into the Teensy's USB
host port. Both are implementations of the same :class:`InputSource` interface,
so the rest of the system does not know or care which one is in use -- it is a
single line in system.yaml.
"""

from .base import ControllerFrame, EdgeTracker, InputSource, InputUnavailable

__all__ = ["ControllerFrame", "EdgeTracker", "InputSource", "InputUnavailable"]


def create_input_source(source: str, *args, **kwargs) -> InputSource:
    """Factory dispatching on ``input.source`` from system.yaml."""
    if source == "mac_gamepad":
        from .mac_gamepad import MacGamepadSource

        return MacGamepadSource(*args, **kwargs)
    if source == "teensy_host":
        from .teensy_host import TeensyHostSource

        return TeensyHostSource(*args, **kwargs)
    raise InputUnavailable(f"unknown input source {source!r}")
