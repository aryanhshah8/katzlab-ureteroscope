"""Controller read directly on the MacBook via pygame.

This is the backend for the controller plugged into the Mac. It maps physical
axis and button indices -- which differ between controllers -- onto the logical
control names through ``controller.yaml``.
"""

from __future__ import annotations

import logging

from ..config import Binding, ControllerConfig
from .base import ControllerFrame, InputSource, InputUnavailable, apply_deadzone_and_curve
from .sdl import NOT_FOUND_HELP, init_joysticks

log = logging.getLogger(__name__)

# Logical axis name -> the controller.yaml axis binding that drives it.
#   left stick  X -> linear travel
#   right stick X -> rotation
#   right stick Y -> flexion
AXIS_ORDER = ("linear", "rotation", "flexion")


class MacGamepadSource(InputSource):
    """Reads a USB or Bluetooth gamepad attached to this Mac."""

    def __init__(
        self,
        controller: ControllerConfig,
        *,
        deadzone: float = 0.06,
        expo: float = 0.35,
        joystick_index: int = 0,
        axis_expo: dict[str, float] | None = None,
        axis_deadzone: dict[str, float] | None = None,
    ) -> None:
        self._controller = controller
        self._deadzone = deadzone
        self._expo = expo
        self._axis_expo = axis_expo or {}
        self._axis_deadzone = axis_deadzone or {}
        self._joystick_index = joystick_index
        self._pygame = None
        self._joystick = None
        self._name = "no controller"
        self._backend = "unknown"

    @property
    def name(self) -> str:
        return self._name

    # -- lifecycle ---------------------------------------------------------

    def open(self) -> None:
        try:
            pygame, count, backend = init_joysticks()
        except ImportError as exc:  # pragma: no cover - environment problem
            raise InputUnavailable(
                "pygame is not installed. Run: pip install -r requirements.txt"
            ) from exc

        self._pygame = pygame
        self._backend = backend

        if count == 0:
            raise InputUnavailable(NOT_FOUND_HELP)
        if self._joystick_index >= count:
            raise InputUnavailable(
                f"controller index {self._joystick_index} requested but only "
                f"{count} controller(s) present"
            )

        self._joystick = pygame.joystick.Joystick(self._joystick_index)
        self._joystick.init()
        self._name = self._joystick.get_name()

        log.info(
            "controller: %s (%d axes, %d buttons, SDL %s backend)",
            self._name,
            self._joystick.get_numaxes(),
            self._joystick.get_numbuttons(),
            self._backend,
        )

        self._verify_matches_calibration()

    def _verify_matches_calibration(self) -> None:
        """Refuse to run against a layout the mapping was not made for.

        This pad enumerates through two different SDL backends depending on
        whether macOS's GameController framework grabs it first, and the two
        disagree about BOTH the name and the axis/button counts. A mapping
        captured on one backend points at entirely the wrong physical buttons on
        the other -- so e-stop stops being e-stop. That has to be a hard failure,
        not a warning: a mis-mapped e-stop is worse than no controller at all.
        """
        expected = self._controller
        axes = self._joystick.get_numaxes()
        buttons = self._joystick.get_numbuttons()

        problems = []
        if expected.name not in ("", "unknown") and expected.name != self._name:
            problems.append(f"name: calibrated {expected.name!r}, connected {self._name!r}")

        highest_axis = max((b.index for b in expected.axes.values()), default=-1)
        highest_button = max(
            (b.index for b in expected.buttons.values() if b.source == "button"), default=-1
        )
        if highest_axis >= axes:
            problems.append(f"mapping needs axis {highest_axis}, device has {axes}")
        if highest_button >= buttons:
            problems.append(f"mapping needs button {highest_button}, device has {buttons}")

        if expected.guid and expected.guid != self._joystick.get_guid():
            problems.append(
                f"guid: calibrated {expected.guid}, connected {self._joystick.get_guid()}"
            )

        if not problems:
            return

        raise InputUnavailable(
            "controller does not match config/controller.yaml:\n  "
            + "\n  ".join(problems)
            + "\n\nThe button indices in that mapping point at different physical\n"
            "buttons on this layout -- E-STOP WOULD NOT BE E-STOP. Refusing to run.\n\n"
            "This pad enumerates differently depending on whether macOS's\n"
            "GameController framework claims it. Either unplug and replug until it\n"
            f"comes back as {expected.name!r}, or re-run:\n"
            "  python scripts/calibrate_controller.py"
        )

    def close(self) -> None:
        if self._joystick is not None:
            try:
                self._joystick.quit()
            except Exception:  # noqa: BLE001 - teardown must not raise
                pass
            self._joystick = None
        if self._pygame is not None:
            self._pygame.joystick.quit()
            self._pygame.quit()
            self._pygame = None

    # -- reading -----------------------------------------------------------

    def poll(self) -> ControllerFrame:
        if self._pygame is None or self._joystick is None:
            raise InputUnavailable("input source is not open")

        # Draining the event queue is what refreshes the joystick state.
        self._pygame.event.pump()

        if not self._is_still_attached():
            return ControllerFrame(connected=False)

        axes = {
            name: self._read_axis(
                self._controller.axes[name],
                self._axis_expo.get(name, self._expo),
                self._axis_deadzone.get(name, self._deadzone),
            )
            for name in AXIS_ORDER
            if name in self._controller.axes
        }
        buttons = {
            name: self._read_button(binding)
            for name, binding in self._controller.buttons.items()
        }
        return ControllerFrame(axes=axes, buttons=buttons, connected=True)

    def _is_still_attached(self) -> bool:
        try:
            return self._pygame.joystick.get_count() > self._joystick_index
        except Exception:  # noqa: BLE001 - a yanked cable can raise anything
            return False

    def _read_axis(
        self,
        binding: Binding,
        expo: float | None = None,
        deadzone: float | None = None,
    ) -> float:
        if binding.source == "button":
            # A d-pad or digital shoulder standing in for an analogue axis.
            raw = 1.0 if self._raw_button(binding.index) else 0.0
        else:
            raw = self._raw_axis(binding.index)

        if binding.invert:
            raw = -raw
        return apply_deadzone_and_curve(
            raw,
            self._deadzone if deadzone is None else deadzone,
            self._expo if expo is None else expo,
        )

    def _read_button(self, binding: Binding) -> bool:
        if binding.source == "button":
            return self._raw_button(binding.index)

        # An analogue trigger reported as an axis. Resting position is -1.0 on
        # most controllers and 0.0 on some, so compare against the threshold
        # after folding the -1..1 range onto 0..1.
        raw = self._raw_axis(binding.index)
        if binding.invert:
            raw = -raw
        normalised = (raw + 1.0) / 2.0
        return normalised >= binding.threshold

    def _raw_axis(self, index: int) -> float:
        if index >= self._joystick.get_numaxes():
            return 0.0
        return float(self._joystick.get_axis(index))

    def _raw_button(self, index: int) -> bool:
        if index >= self._joystick.get_numbuttons():
            return False
        return bool(self._joystick.get_button(index))
