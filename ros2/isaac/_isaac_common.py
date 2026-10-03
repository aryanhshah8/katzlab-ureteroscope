"""Shared Isaac Sim plumbing for the katzlab scripts in this directory:
URDF import, the camera mount, and the placeholder anatomy scene. One copy,
used by ``load_rig.py``, ``preview_scene.py``, and
``generate_training_data.py`` -- so the scene those three scripts agree on
can't quietly drift apart between a one-off viewer, the thing you look at
before committing to a dataset run, and the thing that actually produces
the dataset.

Import this ONLY after ``SimulationApp(...)`` has already been constructed in
the entry script -- it imports ``isaacsim``/``omni`` submodules at module
scope, which only work once the app is booted. See any of the three scripts
above for the pattern.

UNVERIFIED, same caveat as everywhere else in ros2/isaac: written against
NVIDIA's documented Isaac Sim 6.x API, not run against a real install. See
the module docstring in load_rig.py for the two likeliest breakage points
(importer module path, ImportConfig field names) -- they apply here too,
since this is the same import code, just no longer duplicated across scripts.
"""

from __future__ import annotations

import math
from pathlib import Path

URDF_PATH = (
    Path(__file__).resolve().parent.parent / "katzlab_bridge" / "urdf" / "ureteroscope.urdf"
)

# ---------------------------------------------------------------------------
# PLACEHOLDER camera intrinsics -- see CAMERA-SPECS.md for the real-world
# measurements that should eventually replace these. Borrowed from a
# published spec for this class of device (Tongji IROS 2025 ureteroscope
# camera: 1920x1080, 120 deg FOV), NOT measured off this rig's actual camera.
# ---------------------------------------------------------------------------
CAMERA_RESOLUTION = (1920, 1080)       # px, placeholder
CAMERA_HORIZONTAL_APERTURE_MM = 2.0    # placeholder sensor width, matched to
                                        # CAMERA_FOV_DEG below via focal length
CAMERA_FOV_DEG = 120.0                 # placeholder
CAMERA_CLIPPING_RANGE_M = (0.001, 0.05)  # 1mm..50mm -- a lumen is tiny

# ---------------------------------------------------------------------------
# Placeholder scene scale/appearance. See CAMERA-SPECS.md items 4-5.
# ---------------------------------------------------------------------------
ANATOMY_TUBE_RADIUS_RANGE_M = (0.003, 0.006)   # placeholder lumen radius
ANATOMY_LENGTH_M = 0.08
STONE_SCALE_RANGE_M = (0.001, 0.006)     # 1-6mm, roughly clinical calculi sizes
STONE_COLOR_RANGE = ((0.5, 0.5, 0.3), (0.95, 0.9, 0.6))    # yellow/tan-ish
TISSUE_COLOR_RANGE = ((0.6, 0.25, 0.3), (0.9, 0.5, 0.55))  # pink-ish

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


def find_camera_mount_path(stage, rig_prim_path: str) -> str:
    """Resolve camera_link's actual stage path under the imported rig.

    The URDF importer's exact prim-path spelling for a fixed joint + child
    link varies by version, so this tries the two plausible spellings rather
    than hardcoding one. If neither resolves, print the stage tree
    (``for p in stage.Traverse(): print(p.GetPath())``) and see what the
    importer actually named it on your install.
    """
    candidates = [
        f"{rig_prim_path}/flexion_link/camera_joint/camera_link",
        f"{rig_prim_path}/flexion_link/camera_link",
    ]
    for path in candidates:
        if stage.GetPrimAtPath(path).IsValid():
            return path
    raise RuntimeError(
        f"camera_link not found under {rig_prim_path}/flexion_link (tried "
        f"{candidates}) -- check the imported stage tree and add the real "
        "path to the candidates list above"
    )


def create_camera(stage, camera_mount_path: str):
    """Turn camera_link (a plain Xform from the URDF import) into a real
    Isaac camera with PLACEHOLDER intrinsics -- see the CAMERA_* constants
    above and CAMERA-SPECS.md."""
    from pxr import Gf, UsdGeom

    camera = UsdGeom.Camera.Define(stage, camera_mount_path + "/Camera")
    # Focal length that reproduces CAMERA_FOV_DEG given
    # CAMERA_HORIZONTAL_APERTURE_MM, matching USD's pinhole camera convention.
    focal_length_mm = (CAMERA_HORIZONTAL_APERTURE_MM / 2.0) / math.tan(
        math.radians(CAMERA_FOV_DEG) / 2.0
    )
    camera.GetFocalLengthAttr().Set(focal_length_mm)
    camera.GetHorizontalApertureAttr().Set(CAMERA_HORIZONTAL_APERTURE_MM)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(*CAMERA_CLIPPING_RANGE_M))
    # Camera looks down -Z in USD convention; the URDF mount assumes the
    # lumen direction is +X, so rotate -90 deg about Y to point the camera
    # forward along the scope's own +X axis.
    UsdGeom.XformCommonAPI(camera).SetRotate((0, -90, 0))
    return camera


def make_material(stage, path: str, color: tuple[float, float, float]):
    from pxr import Sdf, UsdShade

    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def tag_semantic(stage, prim_path: str, class_name: str) -> None:
    """Attach an Isaac Sim semantic label so a segmentation annotator can
    tell this prim's pixels apart from everything else on the stage."""
    from pxr import Semantics

    prim = stage.GetPrimAtPath(prim_path)
    api = Semantics.SemanticsAPI.Apply(prim, "Semantics")
    api.CreateSemanticTypeAttr().Set("class")
    api.CreateSemanticDataAttr().Set(class_name)


def build_placeholder_anatomy(stage) -> str:
    """A tapered tube (ureter) opening into a wider chamber (calyx).
    Inward-facing normals since the camera sits INSIDE the lumen -- see the
    comment on tube.CreateDoubleSidedAttr() below. Tagged "tissue" for
    segmentation. PLACEHOLDER geometry, see CAMERA-SPECS.md item 4.
    """
    from pxr import UsdGeom, UsdShade

    root_path = "/World/anatomy"
    UsdGeom.Xform.Define(stage, root_path)

    tube_path = root_path + "/ureter_tube"
    tube = UsdGeom.Cylinder.Define(stage, tube_path)
    tube.GetRadiusAttr().Set(sum(ANATOMY_TUBE_RADIUS_RANGE_M) / 2.0)
    tube.GetHeightAttr().Set(ANATOMY_LENGTH_M)
    tube.GetAxisAttr().Set("X")
    # A real lumen is a tube you're INSIDE of, i.e. you need to see its
    # inner wall. UsdGeom.Cylinder's default normals face outward; flip the
    # orientation so Isaac's renderer draws the inside face instead of
    # culling it. (If this doesn't visually hold on a real install, check
    # doubleSided on the mesh in the USD inspector first.)
    tube.CreateDoubleSidedAttr().Set(True)

    calyx_path = root_path + "/calyx_chamber"
    calyx = UsdGeom.Sphere.Define(stage, calyx_path)
    calyx.GetRadiusAttr().Set(ANATOMY_TUBE_RADIUS_RANGE_M[1] * 2.5)
    calyx.CreateDoubleSidedAttr().Set(True)
    UsdGeom.XformCommonAPI(calyx).SetTranslate((ANATOMY_LENGTH_M / 2.0, 0, 0))

    material = make_material(stage, root_path + "/tissue_material", TISSUE_COLOR_RANGE[0])
    UsdShade.MaterialBindingAPI(tube).Bind(material)
    UsdShade.MaterialBindingAPI(calyx).Bind(material)

    tag_semantic(stage, root_path, "tissue")
    return root_path
