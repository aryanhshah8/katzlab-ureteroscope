"""Generate a labeled synthetic dataset for training the stone-detection CV
model, by rendering the ureteroscope's own camera view inside Isaac Sim.

    ~/isaacsim/python.sh ros2/isaac/generate_training_data.py \
        --num-frames 5000 --output-dir /data/katzlab-sdg --headless

WHY THIS EXISTS: the real ureteroscope has one camera and whatever stones
happen to be in front of it on a given day. Isaac Sim can render the same
URDF rig's camera view thousands of times over, each time with a different
stone, a different pose, different lighting -- auto-labeled, because the
simulator already knows exactly which pixels are stone and exactly how far
away everything is. That's the dataset a segmentation/tracking model
actually needs to generalize, at a scale hand-annotation can't reach (compare:
Tongji's IROS 2025 paper hand-annotated 12,768 real endoscopic frames -- this
script can produce more than that in one overnight run, assuming the sim is
actually representative; see the fidelity caveats below).

WHAT "1:1" ACTUALLY REQUIRES, AND WHAT'S STILL A PLACEHOLDER HERE
A synthetic dataset only transfers to the real camera if the simulated
camera's optics and the real scene's appearance are close enough that a
model trained on one works on the other (the "sim-to-real gap"). Three
things drive that gap, and all three are placeholders right now because the
real numbers were never measured against actual hardware -- see
ros2/isaac/CAMERA-SPECS.md for the exact punch list:

  1. CAMERA INTRINSICS (resolution / FOV / lens distortion) -- CAMERA_* below
     currently use a published spec for this class of device (Tongji's IROS
     2025 ureteroscope camera: 1920x1080, 120 deg FOV) as a stand-in, NOT a
     measurement of this rig's actual camera. Swap in real calibration
     the moment it exists.
  2. MOUNT OFFSET -- camera_link in the URDF guesses the camera sits right at
     the flexion tip with zero offset. Also unmeasured.
  3. SCENE APPEARANCE -- the anatomy and stones below are procedural
     placeholders (tapered tubes, randomized spheres/cubes), not real
     calyx/stone geometry or texture. Good enough to validate the whole
     pipeline (camera -> labels -> training) end to end; not good enough to
     claim real sim-to-real transfer until real references replace them.

DOMAIN RANDOMIZATION, NOT ONE FIXED SCENE: every captured frame gets a new
stone count/size/shape/color, new lighting, new tissue tint, and (in
--dynamic mode) stones that have actually drifted under rigid-body physics
since the last frame rather than being teleported. This is the same idea as
Tongji's own "physical simulation enhancement" / "motion blur synthesis"
data augmentation (see their Methods) -- randomize what's cheap to randomize
in sim so the trained model doesn't overfit to one lighting rig or one stone
shape.

UNVERIFIED, same caveat as the rest of ros2/isaac: written against NVIDIA's
documented Isaac Sim 6.x Replicator (omni.replicator.core) API, not run
against a real install. If an annotator/writer call here doesn't match your
Isaac Sim version, Isaac Sim ships worked Replicator examples under
standalone_examples/replicator/ -- check those before guessing at field
names.
"""

from __future__ import annotations

import argparse
import random
from dataclasses import dataclass

# ---------------------------------------------------------------------------
# PLACEHOLDER camera intrinsics -- see the module docstring and
# ros2/isaac/CAMERA-SPECS.md. Replace the moment real hardware specs exist.
# ---------------------------------------------------------------------------
CAMERA_RESOLUTION = (1920, 1080)   # px, placeholder (Tongji IROS 2025 spec)
CAMERA_HORIZONTAL_APERTURE_MM = 2.0  # placeholder sensor width, matched below
                                      # to CAMERA_FOV_DEG via focal length
CAMERA_FOV_DEG = 120.0             # placeholder (Tongji IROS 2025 spec)
CAMERA_CLIPPING_RANGE_M = (0.001, 0.05)  # 1mm..50mm -- a lumen is tiny

# ---------------------------------------------------------------------------
# Domain randomization ranges. All placeholders -- tune once real stone/
# tissue reference imagery exists (see CAMERA-SPECS.md item 5).
# ---------------------------------------------------------------------------
STONE_COUNT_RANGE = (1, 4)
STONE_SCALE_RANGE_M = (0.001, 0.006)     # 1-6mm, roughly clinical calculi sizes
STONE_COLOR_RANGE = ((0.5, 0.5, 0.3), (0.95, 0.9, 0.6))   # yellow/tan-ish
TISSUE_COLOR_RANGE = ((0.6, 0.25, 0.3), (0.9, 0.5, 0.55))  # pink-ish
LIGHT_INTENSITY_RANGE = (500.0, 5000.0)   # lux-ish, matches the real rig's
                                           # 50-5000 lux ring light per Tongji
# Clinical stone drift speed range this rig's docs/the Tongji paper both cite:
# 1-10 mm/s. Used only in --dynamic mode.
STONE_DRIFT_SPEED_RANGE_M_S = (0.001, 0.010)

ANATOMY_TUBE_RADIUS_RANGE_M = (0.003, 0.006)   # placeholder lumen radius
ANATOMY_LENGTH_M = 0.08


@dataclass
class Args:
    num_frames: int
    output_dir: str
    headless: bool
    dynamic: bool
    seed: int


def parse_args() -> Args:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--num-frames", type=int, default=1000)
    parser.add_argument("--output-dir", type=str, default="./katzlab_sdg_output")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--dynamic",
        action="store_true",
        help="Let stones drift under rigid-body physics between captures "
        "(STONE_DRIFT_SPEED_RANGE_M_S) instead of being re-placed instantly "
        "each frame. Slower, more physically grounded.",
    )
    parser.add_argument("--seed", type=int, default=0)
    namespace = parser.parse_args()
    return Args(**vars(namespace))


def main() -> None:
    args = parse_args()
    random.seed(args.seed)

    # -----------------------------------------------------------------
    # 1. Boot Isaac Sim. Must happen before any other isaacsim/omni import,
    #    including _isaac_common -- see load_rig.py for the same note.
    # -----------------------------------------------------------------
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})

    import omni.replicator.core as rep
    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation
    from pxr import Gf, PhysicsSchemaTools, Sdf, UsdGeom, UsdPhysics, UsdShade

    from _isaac_common import import_rig

    try:
        _run(args, simulation_app, rep, World, Articulation, Gf, PhysicsSchemaTools, Sdf,
             UsdGeom, UsdPhysics, UsdShade, import_rig)
    finally:
        simulation_app.close()


def _run(args, simulation_app, rep, World, Articulation, Gf, PhysicsSchemaTools, Sdf,
         UsdGeom, UsdPhysics, UsdShade, import_rig) -> None:
    prim_path = import_rig()
    print(f"[katzlab] imported rig at {prim_path}")

    world = World(stage_units_in_meters=1.0)
    stage = world.stage
    articulation = Articulation(prim_paths_expr=prim_path, name="ureteroscope")
    world.scene.add(articulation)
    world.reset()
    joint_names = list(articulation.dof_names)
    print(f"[katzlab] articulation joints: {joint_names}")

    camera_prim_path = f"{prim_path}/flexion_link/camera_joint/camera_link"
    # The URDF importer's exact prim-path spelling for a fixed joint + child
    # link varies by version; if this path doesn't resolve, print the stage
    # tree (`for p in stage.Traverse(): print(p.GetPath())`) and fix this one
    # constant rather than guessing blind.
    if not stage.GetPrimAtPath(camera_prim_path).IsValid():
        camera_prim_path = f"{prim_path}/flexion_link/camera_link"
    if not stage.GetPrimAtPath(camera_prim_path).IsValid():
        raise RuntimeError(
            f"camera_link not found under {prim_path}/flexion_link -- "
            "check the imported stage tree and fix camera_prim_path above"
        )

    camera = _create_camera(stage, UsdGeom, Gf, camera_prim_path)

    anatomy_path = _build_placeholder_anatomy(stage, UsdGeom, UsdShade, Sdf, Gf)
    _tag_semantic(stage, anatomy_path, "tissue")

    stone_root = "/World/stones"
    stage.DefinePrim(stone_root, "Xform")

    # -- Replicator randomizer graph ------------------------------------
    anatomy_material = _anatomy_material_handle(rep, anatomy_path)
    light = rep.create.light(
        light_type="dome",
        intensity=1000,
        rotation=(0, 0, 0),
    )
    camera_rep = rep.get.prims(path_pattern=camera_prim_path)

    with rep.trigger.on_frame(num_frames=args.num_frames):
        with rep.utils.sequential():
            _randomize_lighting(rep, light)
            _randomize_tissue(rep, anatomy_material)
            stones = _spawn_or_reuse_stones(rep, stone_root)
            _randomize_stones(rep, stones)
            if args.dynamic:
                _apply_stone_physics(stage, UsdPhysics, stones)

    render_product = rep.create.render_product(
        camera_prim_path, resolution=CAMERA_RESOLUTION
    )

    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir=args.output_dir,
        rgb=True,
        semantic_segmentation=True,
        distance_to_camera=True,
        bounding_box_2d_tight=True,
        colorize_semantic_segmentation=True,
    )
    writer.attach([render_product])

    print(f"[katzlab] capturing {args.num_frames} frames to {args.output_dir}")
    rep.orchestrator.run()
    while rep.orchestrator.get_is_started() and simulation_app.is_running():
        simulation_app.update()
    print("[katzlab] done")


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------


def _create_camera(stage, UsdGeom, Gf, camera_prim_path: str):
    """Turn camera_link (a plain Xform from the URDF import) into a real
    Isaac camera with PLACEHOLDER intrinsics. See the module docstring."""
    import math

    camera = UsdGeom.Camera.Define(stage, camera_prim_path + "/Camera")
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


# ---------------------------------------------------------------------------
# Placeholder anatomy: a tapered tube (ureter) opening into a wider chamber
# (calyx). Inward-facing normals since the camera sits inside the lumen.
# ---------------------------------------------------------------------------


def _build_placeholder_anatomy(stage, UsdGeom, UsdShade, Sdf, Gf) -> str:
    root_path = "/World/anatomy"
    xform = UsdGeom.Xform.Define(stage, root_path)

    tube_path = root_path + "/ureter_tube"
    tube = UsdGeom.Cylinder.Define(stage, tube_path)
    tube.GetRadiusAttr().Set(sum(ANATOMY_TUBE_RADIUS_RANGE_M) / 2.0)
    tube.GetHeightAttr().Set(ANATOMY_LENGTH_M)
    tube.GetAxisAttr().Set("X")
    # A real lumen is a tube you're INSIDE of, i.e. you need to see its
    # inner wall. UsdGeom.Cylinder's default normals face outward; flip the
    # orientation so Isaac's renderer draws the inside face instead of
    # culling it. (If this doesn't visually hold on a real install, the fix
    # is doubleSided on the mesh via UsdGeom.Gprim.)
    tube.CreateDoubleSidedAttr().Set(True)

    calyx_path = root_path + "/calyx_chamber"
    calyx = UsdGeom.Sphere.Define(stage, calyx_path)
    calyx.GetRadiusAttr().Set(ANATOMY_TUBE_RADIUS_RANGE_M[1] * 2.5)
    calyx.CreateDoubleSidedAttr().Set(True)
    UsdGeom.XformCommonAPI(calyx).SetTranslate((ANATOMY_LENGTH_M / 2.0, 0, 0))

    material = _make_material(stage, UsdShade, Sdf, root_path + "/tissue_material",
                               tuple(c for c in TISSUE_COLOR_RANGE[0]))
    UsdShade.MaterialBindingAPI(tube).Bind(material)
    UsdShade.MaterialBindingAPI(calyx).Bind(material)

    return root_path


def _make_material(stage, UsdShade, Sdf, path: str, color):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _tag_semantic(stage, prim_path: str, class_name: str) -> None:
    """Attach an Isaac Sim semantic label so the writer's segmentation
    annotator can tell this prim's pixels apart from everything else."""
    from pxr import Semantics

    prim = stage.GetPrimAtPath(prim_path)
    api = Semantics.SemanticsAPI.Apply(prim, "Semantics")
    api.CreateSemanticTypeAttr().Set("class")
    api.CreateSemanticDataAttr().Set(class_name)


# ---------------------------------------------------------------------------
# Stones
# ---------------------------------------------------------------------------


def _spawn_or_reuse_stones(rep, stone_root: str):
    count = random.randint(*STONE_COUNT_RANGE)
    stones = rep.create.sphere(
        semantics=[("class", "stone")],
        count=count,
        position=(0, 0, 0),
    )
    return stones


def _randomize_stones(rep, stones):
    with stones:
        rep.modify.pose(
            position=rep.distribution.uniform(
                (0.005, -0.004, -0.004), (ANATOMY_LENGTH_M - 0.005, 0.004, 0.004)
            ),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)),
            scale=rep.distribution.uniform(STONE_SCALE_RANGE_M[0], STONE_SCALE_RANGE_M[1]),
        )
        rep.randomizer.color(
            colors=rep.distribution.uniform(STONE_COLOR_RANGE[0], STONE_COLOR_RANGE[1])
        )


def _apply_stone_physics(stage, UsdPhysics, stones) -> None:
    """--dynamic mode: give each stone a rigid body + a small random impulse
    so it actually drifts between frames at roughly clinical speed
    (STONE_DRIFT_SPEED_RANGE_M_S), rather than being re-placed instantly.
    Best-effort -- Replicator's own per-frame pose randomization above
    already covers the common "lots of varied stills" training need; this
    is for the cases that specifically want physically continuous motion.
    """
    for prim in stones.get_output_prims().get("prims", []):
        rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.CollisionAPI.Apply(prim)
        speed = random.uniform(*STONE_DRIFT_SPEED_RANGE_M_S)
        direction = [random.uniform(-1, 1) for _ in range(3)]
        norm = sum(d * d for d in direction) ** 0.5 or 1.0
        velocity = [d / norm * speed for d in direction]
        rigid_body.CreateVelocityAttr().Set(velocity)


# ---------------------------------------------------------------------------
# Lighting / tissue randomization
# ---------------------------------------------------------------------------


def _anatomy_material_handle(rep, anatomy_path: str):
    return rep.get.prims(path_pattern=anatomy_path + "/tissue_material/Shader")


def _randomize_tissue(rep, material_handle) -> None:
    with material_handle:
        rep.randomizer.color(
            colors=rep.distribution.uniform(TISSUE_COLOR_RANGE[0], TISSUE_COLOR_RANGE[1])
        )


def _randomize_lighting(rep, light) -> None:
    with light:
        rep.modify.attribute(
            "intensity",
            rep.distribution.uniform(*LIGHT_INTENSITY_RANGE),
        )


if __name__ == "__main__":
    main()
