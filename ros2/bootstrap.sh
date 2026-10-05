#!/usr/bin/env bash
#
# One command to get a Mint box ready for everything under ros2/: ROS 2
# Jazzy, the katzlab_bridge package built, and (if Isaac Sim is found) the
# Python packages the reach-task training needs. Chains together the
# individual steps documented in ros2/README.md and
# ../docs/MINT-ONLY-GUIDE.pdf rather than replacing them -- read those for
# what each step actually does and how to debug it if something here fails.
#
# Run from the repo root:
#     ./ros2/bootstrap.sh
#
# Safe to re-run -- every step it calls is itself safe to re-run
# (setup-mint.sh says so explicitly; colcon build and pip install are
# idempotent by nature).

set -euo pipefail

GREEN=$'\033[32m'; YELLOW=$'\033[33m'; RED=$'\033[31m'; BOLD=$'\033[1m'; OFF=$'\033[0m'
step() { echo; echo "${BOLD}==> $*${OFF}"; }
ok()   { echo "  ${GREEN}ok${OFF}   $*"; }
warn() { echo "  ${YELLOW}warn${OFF} $*"; }
die()  { echo "  ${RED}FAIL${OFF} $*" >&2; exit 1; }

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

[ -f ros2/setup-mint.sh ] || die "run this from a checkout that has ros2/setup-mint.sh (repo root: $REPO_ROOT)"

# ---------------------------------------------------------------------------
step "1. ROS 2 Jazzy + colcon + foxglove-bridge"
# ---------------------------------------------------------------------------
./ros2/setup-mint.sh

# ---------------------------------------------------------------------------
step "2. Re-source ROS 2 in THIS shell"
# ---------------------------------------------------------------------------
# setup-mint.sh only adds the sourcing line to ~/.bashrc for future
# terminals -- this script needs it right now to run colcon in step 3.
if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  ok "sourced /opt/ros/jazzy/setup.bash for this script"
else
  die "ROS 2 Jazzy not found at /opt/ros/jazzy after setup-mint.sh -- check its output above for errors"
fi

# ---------------------------------------------------------------------------
step "3. Build katzlab_bridge"
# ---------------------------------------------------------------------------
(
  cd ros2
  colcon build --packages-select katzlab_bridge
)
ok "built. Every NEW terminal from here on needs:"
echo "       source $REPO_ROOT/ros2/install/setup.bash"

# ---------------------------------------------------------------------------
step "4. Sanity tests (no ROS 2/Isaac Sim needed for these -- just Python)"
# ---------------------------------------------------------------------------
python3 -m pytest ros2/katzlab_bridge/test/ -q || warn "bridge tests failed -- see output above"
python3 -m pytest ros2/isaac/training/test_reach_math.py -q || warn "reach-math tests failed -- see output above"

# ---------------------------------------------------------------------------
step "5. Isaac Sim Python packages (training deps), if Isaac Sim is found"
# ---------------------------------------------------------------------------
# Isaac Sim's own Python (not the system one) is what training actually
# runs under -- gymnasium/numpy/stable-baselines3 have to be installed
# into THAT interpreter, not a venv or /usr/bin/python3. Checked in the
# common install locations; override with ISAACSIM_PYTHON if yours differs.
ISAACSIM_PYTHON="${ISAACSIM_PYTHON:-}"
if [ -z "$ISAACSIM_PYTHON" ]; then
  for candidate in "$HOME/isaacsim/python.sh" "$HOME/.local/share/ov/pkg"/isaac-sim-*/python.sh; do
    if [ -x "$candidate" ]; then
      ISAACSIM_PYTHON="$candidate"
      break
    fi
  done
fi

if [ -n "$ISAACSIM_PYTHON" ] && [ -x "$ISAACSIM_PYTHON" ]; then
  ok "found Isaac Sim Python at $ISAACSIM_PYTHON"
  "$ISAACSIM_PYTHON" -m pip install --quiet gymnasium numpy stable-baselines3
  ok "installed gymnasium, numpy, stable-baselines3 into Isaac Sim's Python"
else
  warn "Isaac Sim not found (looked for ~/isaacsim/python.sh and ~/.local/share/ov/pkg/isaac-sim-*/python.sh)"
  warn "install Isaac Sim itself first (ros2/README.md section 2), then re-run this script,"
  warn "or set ISAACSIM_PYTHON=/path/to/python.sh and re-run just this step:"
  echo "       ISAACSIM_PYTHON=/path/to/python.sh ./ros2/bootstrap.sh"
fi

# ---------------------------------------------------------------------------
step "Done"
# ---------------------------------------------------------------------------
echo
echo "Next: open a NEW terminal (so ROS 2 sourcing from ~/.bashrc takes effect), then"
echo "  source $REPO_ROOT/ros2/install/setup.bash"
echo "  ros2 launch katzlab_bridge sim.launch.py dry_run:=true"
echo
echo "Full walkthrough from here: docs/MINT-ONLY-GUIDE.pdf"
