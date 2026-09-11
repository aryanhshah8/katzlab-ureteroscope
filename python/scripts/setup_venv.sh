#!/usr/bin/env bash
#
# Creates the venv and installs dependencies -- the one thing that genuinely
# CANNOT be copied from another machine. A .venv folder is not portable: its
# interpreter is a symlink baked to one specific Python install path, and any
# compiled packages (pygame ships a compiled SDL2) are built for one specific
# OS and CPU architecture. Zipping .venv from this Mac and dropping it onto
# Linux, Windows, or even a different-architecture Mac will not work -- it
# fails with confusing errors, not a clean "not found".
#
# Running this script fresh on the TARGET machine takes under a minute and
# always works, which is the actual fix for "I need this on another computer".
#
# Run from inside the repo's python/ directory:
#     bash scripts/setup_venv.sh
#
# Windows: use docs/WINDOWS-SETUP.txt instead -- venv activation and paths
# differ enough there that this script does not apply as-is.

set -euo pipefail

cd "$(dirname "$0")/.."   # -> python/, regardless of where this was invoked from

if [ ! -f requirements.txt ]; then
  echo "FAIL: requirements.txt not found here." >&2
  echo "      Run this from inside the repo's python/ directory, e.g.:" >&2
  echo "        cd katzlab-ureteroscope/python && bash scripts/setup_venv.sh" >&2
  exit 1
fi

PYTHON=python3
command -v "$PYTHON" >/dev/null 2>&1 || {
  echo "FAIL: python3 not found on this machine." >&2
  echo "      Mac: install from python.org or via Homebrew." >&2
  echo "      Mint/Ubuntu: sudo apt install python3 python3-venv python3-pip" >&2
  exit 1
}

echo "==> Python: $($PYTHON --version)"

echo "==> Creating .venv"
"$PYTHON" -m venv .venv

echo "==> Installing dependencies"
./.venv/bin/pip install --upgrade pip -q
./.venv/bin/pip install -r requirements.txt

echo "==> Verifying everything imports cleanly"
./.venv/bin/python -c "
import katzlab, pygame, serial, yaml
print('    katzlab package + pygame + pyserial + PyYAML: all import cleanly')
"

echo
echo "Done. Run keyboard control with:"
echo "    ./.venv/bin/python scripts/keyboard_control.py"
echo
echo "Or the full gamepad flow:"
echo "    ./.venv/bin/python -m katzlab bringup --skip-flash --yes"
