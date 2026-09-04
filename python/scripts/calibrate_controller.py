#!/usr/bin/env python3
"""Interactive controller calibration.

Physical axis and button indices differ between controllers, and analogue
triggers show up as buttons on some and as axes on others. Rather than hardcode
a guess, this walks the operator through each control once and writes the result
to ``config/controller.yaml``.

Run with the controller connected:

    python scripts/calibrate_controller.py
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from katzlab.config import Binding, ControllerConfig, save_controller_config  # noqa: E402

try:
    from katzlab.input.sdl import NOT_FOUND_HELP, init_joysticks
except ImportError:
    sys.exit("pygame is not installed. Run: pip install -r requirements.txt")

# Bound by main() via init_joysticks(). The capture helpers below are
# module-level and reach for it by name, so it must live here rather than in
# main()'s locals.
pygame = None


# (logical name, operator prompt, is_axis)
AXIS_STEPS = [
    ("linear", "LEFT stick: push it fully LEFT then RIGHT", True),
    ("rotation", "RIGHT stick: push it fully UP then DOWN", True),
    ("flexion", "RIGHT stick: push it fully LEFT then RIGHT", True),
]

BUTTON_STEPS = [
    ("enable_ready", "LEFT SECONDARY trigger (bumper / LB) -- Enable+Ready"),
    ("standby", "LEFT PRIMARY trigger (LT) -- Standby"),
    ("fire", "RIGHT SECONDARY trigger (bumper / RB) -- Fire"),
    ("pause", "RIGHT PRIMARY trigger (RT) -- Pause"),
    ("estop", "X -- the BOTTOM button of the right-hand cluster (E-stop)"),
    ("home", "H -- the RIGHT button of the right-hand cluster (Home)"),
]

MOVE_THRESHOLD = 0.45
TRIGGER_THRESHOLD = 0.8
AXIS_TIMEOUT_S = 30.0
BUTTON_TIMEOUT_S = 30.0

DIM = "\033[2m"
GREEN = "\033[32m"
RESET = "\033[0m"


def snapshot(js) -> tuple[list[float], list[bool]]:
    pygame.event.pump()
    axes = [js.get_axis(i) for i in range(js.get_numaxes())]
    buttons = [bool(js.get_button(i)) for i in range(js.get_numbuttons())]
    return axes, buttons


def wait_for_release(js) -> None:
    """Block until nothing is being pressed, so steps cannot bleed together."""
    while True:
        axes, buttons = snapshot(js)
        if not any(buttons) and all(abs(v) < 0.3 for v in axes):
            return
        time.sleep(0.05)


def _live(text: str) -> None:
    """Overwrite the current terminal line."""
    sys.stdout.write("\r\033[K  " + text)
    sys.stdout.flush()


def capture_axis(js, prompt: str) -> Binding:
    """Watch until one axis has been pushed to both extremes.

    The readout updates live. Without it there is no way to tell a controller
    that is not reporting from a prompt that is simply still waiting -- which is
    the same thing from the operator's side of the screen.
    """
    print(f"\n  {prompt}")
    print(f"  {DIM}Push it to BOTH extremes. Ctrl-C aborts.{RESET}")
    wait_for_release(js)

    baseline, _ = snapshot(js)
    excursion = [0.0] * len(baseline)
    seen_positive = [False] * len(baseline)
    seen_negative = [False] * len(baseline)

    deadline = time.monotonic() + AXIS_TIMEOUT_S
    settle_until = None

    while True:
        axes, _ = snapshot(js)
        for i, value in enumerate(axes):
            delta = value - baseline[i]
            excursion[i] = max(excursion[i], abs(delta))
            if delta > MOVE_THRESHOLD:
                seen_positive[i] = True
            if delta < -MOVE_THRESHOLD:
                seen_negative[i] = True

        best = max(range(len(excursion)), key=lambda i: excursion[i])
        both = seen_positive[best] and seen_negative[best]

        if excursion[best] < 0.05:
            _live(f"waiting for movement... {int(deadline - time.monotonic())}s")
        else:
            got = ("+" if seen_positive[best] else ".") + ("-" if seen_negative[best] else ".")
            _live(
                f"axis {best:<2} travel {excursion[best]:.2f}  "
                f"directions [{got}]  "
                + (f"{GREEN}both ends seen{RESET}" if both else "push the other way")
            )

        if both:
            # Give it a moment in case a neighbouring axis is moving further.
            settle_until = settle_until or time.monotonic() + 0.5
            if time.monotonic() >= settle_until:
                break

        if time.monotonic() > deadline:
            if excursion[best] >= MOVE_THRESHOLD:
                break          # moved, just never reached both extremes
            print()
            raise SystemExit(
                "  No movement detected on any axis.\n"
                "  Run 'python scripts/gamepad_monitor.py' to see whether the\n"
                "  controller is reporting at all."
            )

        time.sleep(0.02)

    best = max(range(len(excursion)), key=lambda i: excursion[i])
    _live(f"{GREEN}axis {best}{RESET} (travel {excursion[best]:.2f})\n")
    return Binding(source="axis", index=best)


def capture_button(js, prompt: str) -> Binding:
    """Watch until a button goes down, or an analogue trigger swings."""
    print(f"\n  Press and HOLD: {prompt}")
    wait_for_release(js)

    baseline_axes, _ = snapshot(js)
    deadline = time.monotonic() + BUTTON_TIMEOUT_S

    while True:
        axes, buttons = snapshot(js)

        for i, down in enumerate(buttons):
            if down:
                _live(f"{GREEN}button {i}{RESET}\n")
                wait_for_release(js)
                return Binding(source="button", index=i)

        for i, value in enumerate(axes):
            if abs(value - baseline_axes[i]) > TRIGGER_THRESHOLD:
                _live(f"{GREEN}analogue trigger on axis {i}{RESET}\n")
                wait_for_release(js)
                return Binding(source="axis", index=i, threshold=0.5)

        # Show partial trigger travel so a half-pressed trigger is visible.
        moved = max(
            ((abs(v - baseline_axes[i]), i) for i, v in enumerate(axes)),
            default=(0.0, -1),
        )
        if moved[0] > 0.1:
            _live(f"axis {moved[1]} moving ({moved[0]:.2f}) -- press it further")
        else:
            _live(f"waiting for a press... {int(deadline - time.monotonic())}s")

        if time.monotonic() > deadline:
            print()
            raise SystemExit(
                "  Nothing detected.\n"
                "  Run 'python scripts/gamepad_monitor.py' to see which index\n"
                "  this control reports as, then set it in config/controller.yaml."
            )

        time.sleep(0.02)


def main() -> int:
    global pygame
    pygame, count, backend = init_joysticks()

    if count == 0:
        sys.exit(NOT_FOUND_HELP)

    js = pygame.joystick.Joystick(0)
    js.init()

    print("=" * 68)
    print("  KATZLAB CONTROLLER CALIBRATION")
    print("=" * 68)
    print(f"  Controller : {js.get_name()}")
    print(f"  Axes       : {js.get_numaxes()}")
    print(f"  Buttons    : {js.get_numbuttons()}")
    print(f"  GUID       : {js.get_guid()}")
    print(f"  SDL backend: {backend}")
    print("=" * 68)
    print("\nFollow each prompt once. Ctrl-C aborts without writing anything.")

    axes: dict[str, Binding] = {}
    buttons: dict[str, Binding] = {}

    try:
        print("\n--- STICKS " + "-" * 56)
        for name, prompt, _ in AXIS_STEPS:
            axes[name] = capture_axis(js, prompt)

        print("\n--- BUTTONS " + "-" * 55)
        for name, prompt in BUTTON_STEPS:
            buttons[name] = capture_button(js, prompt)
    except KeyboardInterrupt:
        print("\n\nAborted. Nothing was written.")
        return 1

    # Screen coordinates put stick-up at -1. Rotation is the vertical axis, and
    # pushing up should read as positive.
    axes["rotation"] = Binding(
        source=axes["rotation"].source, index=axes["rotation"].index, invert=True
    )

    cfg = ControllerConfig(
        name=js.get_name(),
        guid=js.get_guid(),
        axes=axes,
        buttons=buttons,
        backend=backend,
    )
    cfg.validate()
    path = save_controller_config(cfg)

    print("\n" + "=" * 68)
    print(f"  Written to {path}")
    print("=" * 68)
    print("\n  Sticks:")
    for name, b in cfg.axes.items():
        flag = " (inverted)" if b.invert else ""
        print(f"    {name:<14} {b.source} {b.index}{flag}")
    print("\n  Buttons:")
    for name, b in cfg.buttons.items():
        print(f"    {name:<14} {b.source} {b.index}")
    print("\n  If flexion moves the wrong way in use, flip 'invert' on the")
    print("  flexion axis in config/controller.yaml. Same for the others.\n")

    pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
