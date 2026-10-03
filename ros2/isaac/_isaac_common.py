"""Shared Isaac Sim plumbing for the katzlab scripts in this directory.

Import this ONLY after ``SimulationApp(...)`` has already been constructed in
the entry script -- it imports ``isaacsim``/``omni`` submodules at module
scope, which only work once the app is booted. Both ``load_rig.py`` and
``generate_training_data.py`` follow that order; see either for the pattern.

UNVERIFIED, same caveat as everywhere else in ros2/isaac: written against
NVIDIA's documented Isaac Sim 6.x API, not run against a real install. See
the module docstring in load_rig.py for the two likeliest breakage points
(importer module path, ImportConfig field names) -- they apply here too,
since this is the same import code, just no longer duplicated between the
two scripts.
"""

from __future__ import annotations

from pathlib import Path

URDF_PATH = (
    Path(__file__).resolve().parent.parent / "katzlab_bridge" / "urdf" / "ureteroscope.urdf"
)

# Passed onto ImportConfig; see the module docstring above if a field here no
# longer exists on your Isaac Sim version.
URDF_IMPORT_CONFIG_OVERRIDES = {
    "fix_base": True,           # bolted to a stand, not free-floating
    "merge_fixed_joints": False,
    "convex_decomp": False,
    "import_inertia_tensor": False,
    "self_collision": False,
    "default_drive_type": 1,    # position drive, so set_joint_positions holds
    "default_drive_strength": 1.0e4,
    "default_position_drive_damping": 1.0e3,
}

try:
    from isaacsim.asset.importer.urdf import _urdf as urdf_importer
except ImportError:
    # Isaac Sim < 6.0 naming.
    from omni.importer.urdf import _urdf as urdf_importer  # type: ignore[no-redef]


def import_rig(dest_prim_path: str = "/World/ureteroscope") -> str:
    """Import urdf/ureteroscope.urdf and return the imported prim path."""
    if not URDF_PATH.exists():
        raise FileNotFoundError(f"URDF not found at {URDF_PATH}")

    importer = urdf_importer.acquire_urdf_interface()
    import_config = urdf_importer.ImportConfig()
    for field, value in URDF_IMPORT_CONFIG_OVERRIDES.items():
        setattr(import_config, field, value)

    result, _ = importer.parse_urdf(str(URDF_PATH.parent), URDF_PATH.name, import_config)
    importer.import_robot(
        str(URDF_PATH.parent), URDF_PATH.name, result, import_config, dest_prim_path
    )
    return dest_prim_path
