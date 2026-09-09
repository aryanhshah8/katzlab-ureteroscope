"""The input source interface shared by every controller backend."""

from __future__ import annotations

import math
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field


class InputUnavailable(RuntimeError):
    """No controller, or the backend could not start."""


@dataclass(frozen=True)
class ControllerFrame:
    """One sample of controller state, already normalised.

    Axes are -1.0 .. +1.0 with deadzone and response curve applied, signed so
    that positive means: linear forward, rotation clockwise, flexion up.
    Buttons are keyed by logical name, not by physical index.
    """

    axes: dict[str, float] = field(default_factory=dict)
    buttons: dict[str, bool] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.monotonic)
    connected: bool = True

    def axis(self, name: str) -> float:
        return self.axes.get(name, 0.0)

    def button(self, name: str) -> bool:
        return self.buttons.get(name, False)

    @property
    def any_axis_active(self) -> bool:
        return any(abs(v) > 0.0 for v in self.axes.values())


class InputSource(ABC):
    """A source of :class:`ControllerFrame` samples."""

    @abstractmethod
    def open(self) -> None:
        """Acquire the device. Raises :class:`InputUnavailable` on failure."""

    @abstractmethod
    def poll(self) -> ControllerFrame:
        """Return the current state. Never blocks for long."""

    @abstractmethod
    def close(self) -> None:
        """Release the device."""

    @property
    @abstractmethod
    def name(self) -> str:
        """Human-readable device name, for logs and the status line."""

    def __enter__(self) -> "InputSource":
        self.open()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()


class EdgeTracker:
    """Turns a stream of button levels into press and release events.

    Every laser action is edge-triggered -- holding Fire must not re-send ``f``
    sixty times a second -- so this sits between the raw frames and the command
    dispatch.
    """

    def __init__(self) -> None:
        self._previous: dict[str, bool] = {}

    def update(self, buttons: dict[str, bool]) -> tuple[set[str], set[str]]:
        """Return ``(pressed, released)`` since the previous call."""
        pressed: set[str] = set()
        released: set[str] = set()

        for name, is_down in buttons.items():
            was_down = self._previous.get(name, False)
            if is_down and not was_down:
                pressed.add(name)
            elif was_down and not is_down:
                released.add(name)

        self._previous = dict(buttons)
        return pressed, released

    def is_held(self, name: str) -> bool:
        return self._previous.get(name, False)

    def reset(self) -> None:
        """Forget all state, so a reconnect does not fire spurious edges."""
        self._previous.clear()


def apply_deadzone_and_curve(value: float, deadzone: float, expo: float) -> float:
    """Normalise one raw axis reading.

    Below the deadzone the output is exactly zero. Above it, the remaining range
    is rescaled to a full 0..1 so there is no step at the deadzone edge, then an
    expo curve is blended in for fine control near centre -- which matters a lot
    when the thing on the end of the axis is inside a patient.
    """
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0

    rescaled = (magnitude - deadzone) / (1.0 - deadzone)
    rescaled = min(1.0, rescaled)
    shaped = (1.0 - expo) * rescaled + expo * (rescaled ** 3)
    return shaped if value > 0 else -shaped


def apply_radial_deadzone_and_curve(
    x: float, y: float, deadzone: float, expo: float
) -> tuple[float, float]:
    """Shape a two-axis stick as one vector, preserving its direction.

    Applying a deadzone to each axis separately gives a SQUARE dead region, and
    with different deadzones per axis the sides are unequal -- so a diagonal
    push crosses one threshold before the other and the axis with the smaller
    deadzone starts moving first. Push at 45 degrees and you do not get 45
    degrees.

    Here the deadzone and the response curve are applied to the vector's
    MAGNITUDE, and the direction is carried through untouched. The dead region
    becomes a circle, diagonals stay diagonal, and the magnitude is clamped to 1
    so a corner push is not faster than a cardinal one -- which it otherwise is
    by a factor of root two on pads with a square gate.
    """
    magnitude = math.hypot(x, y)
    if magnitude <= deadzone:
        return 0.0, 0.0

    # Direction first, so nothing below can rotate the vector.
    unit_x, unit_y = x / magnitude, y / magnitude

    rescaled = min(1.0, (magnitude - deadzone) / (1.0 - deadzone))
    shaped = (1.0 - expo) * rescaled + expo * (rescaled ** 3)

    return unit_x * shaped, unit_y * shaped
