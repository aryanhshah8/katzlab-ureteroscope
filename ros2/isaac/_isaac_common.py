"""Shared Isaac Sim plumbing for the katzlab scripts in this directory:
loading the real rig CAD, URDF import (placeholder fallback), the camera
mount, and the placeholder anatomy scene. One copy, used by ``load_rig.py``,
``preview_scene.py``, ``generate_training_data.py``, and
``training/reach_task.py`` -- so none of them can quietly drift apart from
what the others are driving.

Import this ONLY after ``SimulationApp(...)`` has already been constructed in
the entry script -- it imports ``isaacsim``/``omni`` submodules at module
scope, which only work once the app is booted. See any of the scripts above
for the pattern.

TWO WAYS TO GET THE RIG ONTO THE STAGE, in order of preference:

  1. ``load_real_rig()`` -- references the actual CAD (assets/glidar_robot/),
     real geometry and real joints, supplied directly rather than built in
     this session. Verified to load correctly and to have an already-correct
     cm->m / axis fix baked in by whoever built it -- checked with plain USD
     (pxr) in this environment, not assumed. See assets/ASSET-NOTES.md for
     exactly what was and wasn't checked, including one open question
     (composed scope length looks short -- verify in the Isaac viewport).
  2. ``import_rig()`` -- the placeholder box/cylinder URDF. Kept as a
     fallback for machines without the (large, binary) real assets checked
     out, and because RViz/robot_state_publisher on the ROS side still wants
     a plain URDF either way.

UNVERIFIED (Isaac Sim execution itself, as opposed to the real-rig USD data
above): written against NVIDIA's documented Isaac Sim 6.x API, not run
against a real install. See the module docstring in load_rig.py for the two
likeliest breakage points on the placeholder-URDF path (importer module
path, ImportConfig field names).
"""

from __future__ import annotations

import math
from pathlib import Path

URDF_PATH = (
    Path(__file__).resolve().parent.parent / "katzlab_bridge" / "urdf" / "ureteroscope.urdf"
)

# ---------------------------------------------------------------------------
# The real rig. See assets/ASSET-NOTES.md for provenance and what's verified.
# ---------------------------------------------------------------------------
REAL_ASSEMBLY_USD = Path(__file__).resolve().parent / "assets" / "glidar_robot" / (
    "URDF-full-Assembly-ver5_2_no_action_graph.usd"
)

REAL_RIG_ROOT = "/World/ureteroscope/URDF_full_Assembly_ver5"
REAL_JOINT_PATHS = {
    "linear": REAL_RIG_ROOT + "/LA_Motor/LA_Slider_Prismatic",
    "rotation": REAL_RIG_ROOT + "/Rotatng_bearng_outer/Rotatng_bearng_inner_rev",
    # NOTE: these live under a SEPARATE sibling prim, "Endoscope_tip" --
    # not nested inside Endoscope/uretroscope_flexible_CY the way the
    # standalone scope file's own internal path (/Rhino/Geometry/Black/...)
    # would suggest. Found by actually traversing the composed stage, not
    # by assumption: Endoscope/uretroscope_flexible_CY is the payloaded
    # full-detail visual scope; Endoscope_tip is a separate, directly-
    # authored (non-payloaded) physics-rigged copy of just the flexible
    # segment -- already correctly scaled/positioned in the assembly's own
    # layer, so it needed none of the payload-loading fix load_real_rig()
    # applies for the other one.
    "flexion_segments": [
        f"{REAL_RIG_ROOT}/Endoscope_tip/Black/mesh{i}/midJoint" for i in range(1, 8)
    ],
}
REAL_TIP_PRIM_PATH = REAL_RIG_ROOT + "/Endoscope_tip/Black/mesh7"

# Real limits -- CAD mechanical limits where measured (linear, rotation),
# IROS 2026 paper Table I otherwise (flexion has none on the CAD's individual
# segment joints; the real system enforces the aggregate limit in software,
# which is what this number represents). See ASSET-NOTES.md for the CAD-vs-
# paper discrepancy on rotation range (CAD is narrower) -- not resolved,
# flagged there.
REAL_LINEAR_RANGE_M = (-0.123, 0.123)
REAL_ROTATION_RANGE_DEG = (-153.5526885986328, 153.5526885986328)
REAL_FLEXION_RANGE_DEG = (-270.0, 270.0)

# Max commanded rates, IROS 2026 paper Table I.
REAL_LINEAR_MAX_RATE_M_S = 0.002
REAL_ROTATION_MAX_RATE_DEG_S = 30.0
REAL_FLEXION_MAX_RATE_DEG_S = 35.0

# Real benchtop stone-target layout, same paper: 2x4 grid, submerged, 5cm
# between rows, 6cm between adjacent columns, center-to-center. Centered on
# the scope's approach direction for lack of a measured absolute origin --
# only the relative spacing is from the paper, the overall placement isn't.
STONE_GRID_ROWS = 2
STONE_GRID_COLS = 4
STONE_GRID_ROW_SPACING_M = 0.05
STONE_GRID_COL_SPACING_M = 0.06

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

def import_rig(dest_prim_path: str = "/World/ureteroscope") -> str:
    """Import urdf/ureteroscope.urdf and return the imported prim path.

    The urdf_importer extension import is deliberately lazy (inside this
    function, not at module scope) so that everything else in this module
    -- in particular the real-rig functions below, which only need plain
    USD -- stays importable and testable outside Isaac Sim, where this
    extension doesn't exist.
    """
    try:
        from isaacsim.asset.importer.urdf import _urdf as urdf_importer
    except ImportError:
        # Isaac Sim < 6.0 naming.
        from omni.importer.urdf import _urdf as urdf_importer  # type: ignore[no-redef]

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


# ---------------------------------------------------------------------------
# The real rig
# ---------------------------------------------------------------------------


def real_rig_available() -> bool:
    return REAL_ASSEMBLY_USD.exists()


def load_real_rig(stage) -> str:
    """Reference the real CAD assembly onto the stage at a fixed mount point
    (/World/ureteroscope) -- fixed, not parameterized, because
    REAL_JOINT_PATHS/REAL_TIP_PRIM_PATH are precomputed constants relative
    to exactly that path; mounting it anywhere else would silently
    desync them. Returns REAL_RIG_ROOT. See assets/ASSET-NOTES.md before
    relying on this for anything beyond "does it load and are the joints
    where expected" -- the composed-scale question there is still open.
    """
    if not real_rig_available():
        raise FileNotFoundError(
            f"real rig assets not found at {REAL_ASSEMBLY_USD} -- "
            "fall back to import_rig() (the placeholder URDF) if these "
            "weren't checked out, or see assets/ASSET-NOTES.md for where "
            "they're supposed to come from"
        )
    from pxr import UsdGeom

    prim = stage.DefinePrim("/World/ureteroscope", "Xform")
    prim.GetReferences().AddReference(str(REAL_ASSEMBLY_USD))
    UsdGeom.Xform(prim)
    # A reference added after the stage already exists introduces new
    # payload arcs (the ureteroscope sub-asset) that the stage's initial
    # load rules -- computed before this reference existed -- don't cover.
    # Without this, the scope's own geometry/joints silently fail to
    # compose in: the Xform prim exists but has no children. Confirmed by
    # actually hitting this (empty children) before adding the explicit
    # Load() call, not assumed.
    stage.Load()
    return REAL_RIG_ROOT


def set_real_linear_target(stage, position_m: float) -> None:
    """Drive the real linear stage's prismatic joint to an absolute position
    (meters, REAL_LINEAR_RANGE_M), by setting its USD physics drive target
    directly -- the same attribute the asset's own authored defaults already
    use (confirmed present when the asset was inspected), rather than going
    through Isaac's Articulation wrapper, which needs the two separate
    articulation roots in this asset (see ASSET-NOTES.md) handled correctly
    and is a less direct, less obviously-correct path for a single joint."""
    position_m = max(REAL_LINEAR_RANGE_M[0], min(REAL_LINEAR_RANGE_M[1], position_m))
    joint = stage.GetPrimAtPath(REAL_JOINT_PATHS["linear"])
    joint.GetAttribute("drive:linear:physics:targetPosition").Set(position_m)


def set_real_rotation_target(stage, angle_deg: float) -> None:
    """Drive the real rotation bearing's revolute joint (degrees)."""
    angle_deg = max(REAL_ROTATION_RANGE_DEG[0], min(REAL_ROTATION_RANGE_DEG[1], angle_deg))
    joint = stage.GetPrimAtPath(REAL_JOINT_PATHS["rotation"])
    joint.GetAttribute("drive:angular:physics:targetPosition").Set(angle_deg)


def set_real_flexion_target(stage, tip_angle_deg: float) -> None:
    """Drive the real flexible tip's 7-segment continuum chain toward a
    single aggregate tip angle (degrees, REAL_FLEXION_RANGE_DEG), by
    distributing it evenly across the 7 unconstrained segment joints --
    each gets tip_angle_deg / 7. This mirrors the same single-scalar
    flexion abstraction katzlab's own Python control already uses
    (flexion_target_deg); it is NOT a real continuum-mechanics solve, just
    the simplest thing that makes "one commanded tip angle" produce a
    plausible-looking bend across all 7 segments rather than concentrating
    it at one joint. Good enough for the reach task's first milestone;
    revisit if the resulting bend shape doesn't look right in the viewport.
    """
    tip_angle_deg = max(REAL_FLEXION_RANGE_DEG[0], min(REAL_FLEXION_RANGE_DEG[1], tip_angle_deg))
    per_segment_deg = tip_angle_deg / len(REAL_JOINT_PATHS["flexion_segments"])
    for joint_path in REAL_JOINT_PATHS["flexion_segments"]:
        joint = stage.GetPrimAtPath(joint_path)
        joint.GetAttribute("drive:angular:physics:targetPosition").Set(per_segment_deg)


def get_real_tip_world_position(stage):
    """World-space position of the scope's distal tip (mesh7), as the
    bounding-box centroid -- NOT the prim's own transform, which reads
    (0,0,0) for these meshes since Rhino-exported geometry bakes position
    into vertex data rather than xformOps (confirmed when the asset was
    inspected). Returns a Gf.Vec3d."""
    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(REAL_TIP_PRIM_PATH)
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    return bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange().GetMidpoint()


def stone_grid_positions(origin_m=(0.0, 0.0, 0.0)):
    """The real benchtop 2x4 stone grid from the IROS 2026 paper (rows along
    Y, columns along Z, spaced per STONE_GRID_*_SPACING_M), offset from
    origin_m. Only the spacing is from the paper -- origin_m is yours to
    place relative to wherever the scope approaches from in your scene.
    Returns a list of (x, y, z) tuples, STONE_GRID_ROWS * STONE_GRID_COLS
    long.
    """
    ox, oy, oz = origin_m
    positions = []
    for row in range(STONE_GRID_ROWS):
        for col in range(STONE_GRID_COLS):
            y = oy + (row - (STONE_GRID_ROWS - 1) / 2.0) * STONE_GRID_ROW_SPACING_M
            z = oz + (col - (STONE_GRID_COLS - 1) / 2.0) * STONE_GRID_COL_SPACING_M
            positions.append((ox, y, z))
    return positions


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
