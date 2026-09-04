#!/usr/bin/env python3
"""Live gamepad readout -- the Python replacement for Standalone_Controller_Diagnostics.ino.

Answers one question: is input actually arriving? Every axis and every button is
redrawn in place, so pressing anything produces visible movement immediately.

    python scripts/gamepad_monitor.py

Use it to read off indices by hand if the calibration wizard is being awkward,
then write them into config/controller.yaml directly.
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

try:
    from katzlab.input.sdl import NOT_FOUND_HELP, init_joysticks
except ImportError:
    sys.exit("pygame is not installed. Run: pip install -r requirements.txt")

HOME = "\033[H"
CLEAR = "\033[2J"
HIDE_CURSOR = "\033[?25l"
SHOW_CURSOR = "\033[?25h"
DIM = "\033[2m"
BOLD = "\033[1m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
RESET = "\033[0m"

BAR_WIDTH = 21          # odd, so there is a true centre cell
ACTIVE_THRESHOLD = 0.15


def bar(value: float) -> str:
    """A -1..+1 value as a centre-anchored bar."""
    value = max(-1.0, min(1.0, value))
    centre = BAR_WIDTH // 2
    cells = ["-"] * BAR_WIDTH
    cells[centre] = "|"

    extent = int(round(abs(value) * centre))
    for i in range(1, extent + 1):
        pos = centre + i if value > 0 else centre - i
        if 0 <= pos < BAR_WIDTH:
            cells[pos] = "#"

    body = "".join(cells)
    colour = GREEN if abs(value) > ACTIVE_THRESHOLD else DIM
    return f"{colour}{body}{RESET}"


def main() -> int:
    pygame, count, backend = init_joysticks()

    if count == 0:
        sys.exit(NOT_FOUND_HELP)

    js = pygame.joystick.Joystick(0)
    js.init()

    axis_count = js.get_numaxes()
    button_count = js.get_numbuttons()
    hat_count = js.get_numhats()

    # Rest position is sampled once so triggers that idle at -1.0 do not look
    # permanently active, and so drift is visible as movement from baseline.
    pygame.event.pump()
    time.sleep(0.2)
    pygame.event.pump()
    baseline = [js.get_axis(i) for i in range(axis_count)]
    peak = [0.0] * axis_count
    ever_pressed = [False] * button_count

    sys.stdout.write(CLEAR + HIDE_CURSOR)
    try:
        while True:
            pygame.event.pump()

            lines = [
                f"{BOLD}GAMEPAD MONITOR{RESET}   {js.get_name()}   "
                f"{axis_count} axes, {button_count} buttons, {hat_count} hat(s)   [SDL {backend}]",
                f"{DIM}Ctrl-C to exit. 'peak' is the largest movement seen since start.{RESET}",
                "",
                f"{BOLD}AXES{RESET}",
            ]

            for i in range(axis_count):
                raw = js.get_axis(i)
                delta = raw - baseline[i]
                peak[i] = max(peak[i], abs(delta))

                marker = f" {YELLOW}<-- MOVING{RESET}" if abs(delta) > ACTIVE_THRESHOLD else ""
                seen = f"{DIM}peak {peak[i]:.2f}{RESET}" if peak[i] > ACTIVE_THRESHOLD else f"{DIM}     -   {RESET}"
                lines.append(
                    f"  axis {i:<2} {bar(delta)} {raw:+.3f}  {seen}{marker}"
                )

            lines += ["", f"{BOLD}BUTTONS{RESET}"]
            row = "  "
            for i in range(button_count):
                down = bool(js.get_button(i))
                if down:
                    ever_pressed[i] = True
                if down:
                    row += f"{GREEN}{BOLD}[{i:>2}]{RESET}"
                elif ever_pressed[i]:
                    row += f"{DIM}({i:>2}){RESET}"
                else:
                    row += f"{DIM} {i:>2} {RESET}"
            lines.append(row)
            lines.append(
                f"  {DIM}[n] held now   (n) pressed at some point   n never pressed{RESET}"
            )

            if hat_count:
                lines += ["", f"{BOLD}HATS{RESET}"]
                for i in range(hat_count):
                    lines.append(f"  hat {i}: {js.get_hat(i)}")

            lines.append("")
            sys.stdout.write(HOME + "\n".join(line + "\033[K" for line in lines) + "\033[J")
            sys.stdout.flush()
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write(SHOW_CURSOR + "\n")
        sys.stdout.flush()
        pygame.quit()

    print("Axes that moved:", [i for i, p in enumerate(peak) if p > ACTIVE_THRESHOLD] or "none")
    print("Buttons pressed:", [i for i, p in enumerate(ever_pressed) if p] or "none")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
