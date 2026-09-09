#!/usr/bin/env python3
"""Live view of the stick values the control loop actually receives.

Shows the RAW pad reading beside the SHAPED value fed to motion, so an axis
that has stopped working can be placed on one side or the other of the shaping
stage rather than guessed at.

    python scripts/check_sticks.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from katzlab.config import load_controller_config, load_system_config
from katzlab.input import create_input_source

BOLD, DIM, GREEN, RED, OFF = "\033[1m", "\033[2m", "\033[32m", "\033[31m", "\033[0m"


def bar(v: float, width: int = 21) -> str:
    v = max(-1.0, min(1.0, v))
    centre = width // 2
    cells = ["-"] * width
    cells[centre] = "|"
    for i in range(1, int(round(abs(v) * centre)) + 1):
        pos = centre + i if v > 0 else centre - i
        if 0 <= pos < width:
            cells[pos] = "#"
    body = "".join(cells)
    return f"{GREEN if abs(v) > 0.02 else DIM}{body}{OFF}"


def main() -> int:
    cfg = load_system_config()
    ctl = load_controller_config()
    src = create_input_source(
        cfg.input.source, ctl,
        deadzone=cfg.input.deadzone, expo=cfg.input.expo,
        axis_expo=cfg.input.axis_expo, axis_deadzone=cfg.input.axis_deadzone,
        paired_axes=cfg.input.paired_axes,
    )
    src.open()

    js = src._joystick                      # raw, for the before/after comparison
    idx = {n: b.index for n, b in ctl.axes.items()}
    peak = {n: 0.0 for n in ("linear", "rotation", "flexion")}

    print(f"\n{BOLD}STICK CHECK{OFF}  {src.name}")
    print(f"{DIM}raw = straight off the pad   shaped = what motion receives{OFF}")
    print(f"{DIM}Move every stick. Ctrl-C when done.{OFF}\n")

    sys.stdout.write("\033[?25l")
    try:
        while True:
            f = src.poll()
            lines = []
            for name in ("linear", "rotation", "flexion"):
                shaped = f.axis(name)
                peak[name] = max(peak[name], abs(shaped))
                raw = js.get_axis(idx[name]) if idx.get(name) is not None else 0.0
                lines.append(
                    f"  {name:<9} raw {raw:+.3f}   shaped {shaped:+.3f}  {bar(shaped)}"
                )
            sys.stdout.write("\033[H\033[J" if False else "")
            sys.stdout.write("\r" + "\n".join(lines) + "\033[3A\r")
            sys.stdout.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        sys.stdout.write("\033[3B\n")
    finally:
        sys.stdout.write("\033[?25h")
        src.close()

    print(f"\n{BOLD}peaks seen{OFF}")
    for name, v in peak.items():
        flag = f"   {RED}<-- NEVER MOVED{OFF}" if v < 0.02 else ""
        print(f"  {name:<9} {v:.3f}{flag}")
    print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
