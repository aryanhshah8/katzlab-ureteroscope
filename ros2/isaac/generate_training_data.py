"""Generate a labeled synthetic dataset for training the stone-detection CV
model, by rendering the ureteroscope's own camera view inside Isaac Sim.

    ~/isaacsim/python.sh ros2/isaac/generate_training_data.py \
        --num-frames 5000 --output-dir /data/katzlab-sdg --headless

RUN preview_scene.py FIRST. This script commits straight to a bulk headless
capture; it does not stop to let you look at what it built. If the camera
is pointed the wrong way or the anatomy is inside-out, you won't find out
until you're looking at 5000 bad frames. preview_scene.py builds the exact
same scene (same shared code, see _isaac_common.py) and just sits there so
you can look around first.

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
model trained on one works on the other (the "sim-to-real gap"). Camera
intrinsics, mount offset, and scene appearance are all placeholders right
now -- see ros2/isaac/CAMERA-SPECS.md for the exact punch list and
_isaac_common.py for where the placeholder constants live.

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

# Replicator/physics-specific randomization ranges -- not shared with
# preview_scene.py, which places stones statically rather than through
# Replicator. Scene-shape placeholders (camera, anatomy, stone size/color)
# live in _isaac_common.py since preview_scene.py needs those too.
STONE_COUNT_RANGE = (1, 4)
LIGHT_INTENSITY_RANGE = (500.0, 5000.0)   # lux-ish, matches the real rig's
                                           # 50-5000 lux ring light per Tongji
# Clinical stone drift speed range this rig's docs/the Tongji paper both cite:
# 1-10 mm/s. Used only in --dynamic mode.
STONE_DRIFT_SPEED_RANGE_M_S = (0.001, 0.010)


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
    from pxr import UsdPhysics

    import _isaac_common as common

    try:
        _run(args, simulation_app, rep, World, Articulation, UsdPhysics, common)
    finally:
        simulation_app.close()


def _run(args, simulation_app, rep, World, Articulation, UsdPhysics, common) -> None:
    prim_path = common.import_rig()
    print(f"[katzlab] imported rig at {prim_path}")

    world = World(stage_units_in_meters=1.0)
    stage = world.stage
    articulation = Articulation(prim_paths_expr=prim_path, name="ureteroscope")
    world.scene.add(articulation)
    world.reset()
    print(f"[katzlab] articulation joints: {list(articulation.dof_names)}")

    camera_mount_path = common.find_camera_mount_path(stage, prim_path)
    common.create_camera(stage, camera_mount_path)
    camera_render_path = camera_mount_path + "/Camera"

    anatomy_path = common.build_placeholder_anatomy(stage)

    stone_root = "/World/stones"
    stage.DefinePrim(stone_root, "Xform")

    # -- Replicator randomizer graph ------------------------------------
    anatomy_material = rep.get.prims(path_pattern=anatomy_path + "/tissue_material/Shader")
    light = rep.create.light(light_type="dome", intensity=1000, rotation=(0, 0, 0))

    with rep.trigger.on_frame(num_frames=args.num_frames):
        with rep.utils.sequential():
            _randomize_lighting(rep, light)
            _randomize_tissue(rep, anatomy_material, common)
            stones = _spawn_stones(rep, stone_root)
            _randomize_stones(rep, stones, common)
            if args.dynamic:
                _apply_stone_physics(UsdPhysics, stones)

    render_product = rep.create.render_product(
        camera_render_path, resolution=common.CAMERA_RESOLUTION
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
# Stones
# ---------------------------------------------------------------------------


def _spawn_stones(rep, stone_root: str):
    count = random.randint(*STONE_COUNT_RANGE)
    return rep.create.sphere(semantics=[("class", "stone")], count=count, position=(0, 0, 0))


def _randomize_stones(rep, stones, common):
    with stones:
        rep.modify.pose(
            position=rep.distribution.uniform(
                (0.005, -0.004, -0.004),
                (common.ANATOMY_LENGTH_M - 0.005, 0.004, 0.004),
            ),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)),
            scale=rep.distribution.uniform(
                common.STONE_SCALE_RANGE_M[0], common.STONE_SCALE_RANGE_M[1]
            ),
        )
        rep.randomizer.color(
            colors=rep.distribution.uniform(
                common.STONE_COLOR_RANGE[0], common.STONE_COLOR_RANGE[1]
            )
        )


def _apply_stone_physics(UsdPhysics, stones) -> None:
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


def _randomize_tissue(rep, material_handle, common) -> None:
    with material_handle:
        rep.randomizer.color(
            colors=rep.distribution.uniform(
                common.TISSUE_COLOR_RANGE[0], common.TISSUE_COLOR_RANGE[1]
            )
        )


def _randomize_lighting(rep, light) -> None:
    with light:
        rep.modify.attribute("intensity", rep.distribution.uniform(*LIGHT_INTENSITY_RANGE))


if __name__ == "__main__":
    main()
