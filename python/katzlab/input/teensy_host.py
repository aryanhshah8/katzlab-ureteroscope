"""Controller read by the Teensy's USB host port, streamed up the serial link.

This is the second deployment shape: the controller stays plugged into the
Teensy (as ``Teensy_Linear_OpenLoop_Controller_Homing.ino`` does today), the
firmware publishes raw axis and button state as a line of telemetry, and Python
still owns every control decision.

The firmware side of this bridge is not written yet, so the class is present to
keep the interface honest and to fail with an explanation rather than an
ImportError. The expected line format is::

    GP <buttons_hex> <axis0> <axis1> <axis2> <axis3> ...

with axes as raw signed integers, exactly what ``joystick.getAxis()`` returns.
"""

from __future__ import annotations

import logging
import re

from ..config import Binding, ControllerConfig
from ..transport import Link
from .base import ControllerFrame, InputSource, InputUnavailable, apply_deadzone_and_curve

log = logging.getLogger(__name__)

RE_GAMEPAD = re.compile(r"^GP\s+([0-9a-fA-F]+)((?:\s+-?\d+)*)\s*$")

# Raw axis span assumed when the firmware has not yet reported a wider one.
DEFAULT_AXIS_HALF_RANGE = 32767.0


class TeensyHostSource(InputSource):
    """Consumes ``GP`` telemetry lines from the firmware."""

    def __init__(
        self,
        controller: ControllerConfig,
        link: Link,
        *,
        deadzone: float = 0.12,
        expo: float = 0.35,
    ) -> None:
        self._controller = controller
        self._link = link
        self._deadzone = deadzone
        self._expo = expo
        self._buttons_raw = 0
        self._axes_raw: list[int] = []
        self._seen_frame = False

    @property
    def name(self) -> str:
        return "Teensy USB host bridge"

    def open(self) -> None:
        if not self._link.is_open:
            raise InputUnavailable(
                "teensy_host input needs an open serial link; open the link "
                "before the input source"
            )
        log.info("listening for GP telemetry frames on %s", self._link.port)

    def close(self) -> None:
        # The link is owned by the caller; nothing to release here.
        return

    def poll(self) -> ControllerFrame:
        for line in self._link.drain():
            if match := RE_GAMEPAD.match(line):
                self._buttons_raw = int(match.group(1), 16)
                self._axes_raw = [int(v) for v in match.group(2).split()]
                self._seen_frame = True

        if not self._seen_frame:
            raise InputUnavailable(
                "no GP telemetry received. The firmware input bridge is not "
                "implemented yet -- set input.source: mac_gamepad in "
                "system.yaml, or flash firmware that publishes GP frames."
            )

        axes = {
            name: self._read_axis(binding)
            for name, binding in self._controller.axes.items()
        }
        buttons = {
            name: self._read_button(binding)
            for name, binding in self._controller.buttons.items()
        }
        return ControllerFrame(axes=axes, buttons=buttons, connected=True)

    def _read_axis(self, binding: Binding) -> float:
        if binding.source == "button":
            raw = 1.0 if self._buttons_raw & (1 << binding.index) else 0.0
        elif binding.index < len(self._axes_raw):
            raw = self._axes_raw[binding.index] / DEFAULT_AXIS_HALF_RANGE
        else:
            raw = 0.0

        raw = max(-1.0, min(1.0, raw))
        if binding.invert:
            raw = -raw
        return apply_deadzone_and_curve(raw, self._deadzone, self._expo)

    def _read_button(self, binding: Binding) -> bool:
        if binding.source == "button":
            return bool(self._buttons_raw & (1 << binding.index))
        if binding.index >= len(self._axes_raw):
            return False
        normalised = self._axes_raw[binding.index] / DEFAULT_AXIS_HALF_RANGE
        return abs(normalised) >= binding.threshold
