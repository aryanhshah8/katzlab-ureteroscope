#!/usr/bin/env python3
"""One command for keyboard-driven control -- nothing to remember.

Equivalent to typing out:

    katzlab --input-source keyboard bringup --skip-flash --yes

every time. Extra arguments are forwarded straight through to `bringup`, so
this still takes all its normal flags:

    python scripts/keyboard_control.py --no-laser
    python scripts/keyboard_control.py --debug-buttons
    python scripts/keyboard_control.py --no-record --no-status

--skip-flash is baked in on purpose: this is the every-day "just go" command,
matching how gamepad use settles into `bringup --skip-flash --yes` after the
first run. For the actual FIRST flash (or after changing the firmware), use
the full command directly so the upload step runs:

    katzlab --input-source keyboard bringup --yes

Needs a real interactive terminal -- see docs/KEYBOARD-CONTROL.md for the key
layout and the held-key limitation before using this for anything that
matters.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from katzlab.cli import main

if __name__ == "__main__":
    raise SystemExit(
        main(["--input-source", "keyboard", "bringup", "--skip-flash", "--yes", *sys.argv[1:]])
    )
