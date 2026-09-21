"""Makes both ``katzlab`` and ``katzlab_bridge`` importable without a colcon
install -- these tests are meant to be runnable with plain pytest, the same
way python/tests/ are, not only after a full ROS build."""

import sys
from pathlib import Path

_PKG_ROOT = Path(__file__).resolve().parents[1]       # ros2/katzlab_bridge
_REPO_ROOT = _PKG_ROOT.parents[1]                      # repo root
_PYTHON_SRC = _REPO_ROOT / "python"

for path in (_PKG_ROOT, _PYTHON_SRC):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
