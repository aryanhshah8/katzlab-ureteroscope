"""Build the scene and just sit there, so you can look at it before
committing to a bulk capture run (generate_training_data.py) or a training
run (training/reach_task.py).

    ~/isaacsim/python.sh ros2/isaac/preview_scene.py                 # real rig (default)
    ~/isaacsim/python.sh ros2/isaac/preview_scene.py --placeholder   # placeholder URDF

REAL RIG MODE (default, if assets/glidar_robot/ is checked out -- see
assets/ASSET-NOTES.md): opens the actual CAD -- linear stage, rotation
bearing, the real Wolf RIWO ureteroscope mesh -- plus the real 8-stone
benchtop target grid from the IROS 2026 paper (still placeholder spheres,
real positions). No camera/anatomy in this mode -- neither exists for the
real rig yet (the real camera mount point is an open item, see
CAMERA-SPECS.md), and the reach task this scene is mostly for doesn't need
either: it's driven by tip/stone *positions*, not vision.

PLACEHOLDER MODE (--placeholder): the original placeholder box/cylinder
URDF scene with a simulated camera and fake tube/calyx anatomy, for the CV
dataset pipeline (generate_training_data.py), which does need a camera.

WHAT TO CHECK BEFORE MOVING ON, real rig mode:
  1. Does the scope visually look the right length relative to the table/
     linear stage? ASSET-NOTES.md flags an unresolved composed-bbox-looks-
     short question -- this is where you'd actually see it.
  2. Do the 7 flexible-tip segments (Endoscope_tip) visually bend when
     driven through their range, and does that look like a plausible
     continuum bend rather than a kink concentrated at one joint?
  3. Are the 8 stone markers positioned sensibly relative to where the
     scope tip actually points?

WHAT TO CHECK, placeholder mode: see the four checks that used to be the
only ones here -- camera direction at rest and through the flexion range,
anatomy wall visible, stones inside the lumen.

UNVERIFIED, same caveat as the rest of ros2/isaac: written against NVIDIA's
documented Isaac Sim 6.x API, not run against a real install. The real-rig
loading/joint-path code in _isaac_common.py is the one exception -- that
part is checked against plain USD (pxr) in this environment; see
assets/ASSET-NOTES.md.
"""

from __future__ import annotations

import argparse
import random
import sys


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--placeholder",
        action="store_true",
        help="Use the placeholder URDF/anatomy/camera scene instead of the real CAD.",
    )
    parser.add_argument(
        "--num-stones", type=int, default=3, help="Placeholder mode only: how many to scatter."
    )
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    random.seed(args.seed)

    # -----------------------------------------------------------------
    # Boot Isaac Sim. Must happen before any other isaacsim/omni import,
    # including _isaac_common -- see load_rig.py for the same note.
    # -----------------------------------------------------------------
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": False})

    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation

    import _isaac_common as common

    try:
        use_real_rig = not args.placeholder and common.real_rig_available()
        if args.placeholder:
            print("[katzlab] --placeholder: using the placeholder scene")
        elif not common.real_rig_available():
            print(
                "[katzlab] real rig assets not found, falling back to the "
                "placeholder scene -- see assets/ASSET-NOTES.md"
            )

        if use_real_rig:
            _run_real(simulation_app, World, common)
        else:
            _run_placeholder(args, simulation_app, World, Articulation, common)
    finally:
        simulation_app.close()


def _run_real(simulation_app, World, common) -> None:
    world = World(stage_units_in_meters=1.0)
    stage = world.stage
    world.scene.add_default_ground_plane()
    common.load_real_rig(stage)
    world.reset()
    print(f"[katzlab] loaded real rig at {common.REAL_RIG_ROOT}")
    print(f"[katzlab] linear joint: {common.REAL_JOINT_PATHS['linear']}")
    print(f"[katzlab] rotation joint: {common.REAL_JOINT_PATHS['rotation']}")
    print(f"[katzlab] flexion tip prim: {common.REAL_TIP_PRIM_PATH}")

    _place_real_stone_grid(stage, common)
    print("[katzlab] placed the real 8-stone benchtop grid (IROS 2026 paper layout)")

    print("[katzlab] scene ready -- window open, close it or Ctrl+C when done looking")
    _spin(simulation_app, world)


def _place_real_stone_grid(stage, common) -> None:
    from pxr import UsdGeom, UsdShade

    root_path = "/World/stones"
    UsdGeom.Xform.Define(stage, root_path)
    material = common.make_material(stage, root_path + "/material", common.STONE_COLOR_RANGE[0])

    # Offset so the grid sits roughly in front of wherever the tip starts --
    # only the spacing is from the paper, this placement is a guess; nudge
    # it once you can see where the tip actually points.
    tip_pos = common.get_real_tip_world_position(stage)
    origin = (tip_pos[0] + 0.05, tip_pos[1], tip_pos[2])

    for i, position in enumerate(common.stone_grid_positions(origin)):
        stone_path = f"{root_path}/stone_{i}"
        stone = UsdGeom.Sphere.Define(stage, stone_path)
        stone.GetRadiusAttr().Set(sum(common.STONE_SCALE_RANGE_M) / 2.0)
        UsdGeom.XformCommonAPI(stone).SetTranslate(position)
        UsdShade.MaterialBindingAPI(stone).Bind(material)
        common.tag_semantic(stage, stone_path, "stone")


def _run_placeholder(args, simulation_app, World, Articulation, common) -> None:
    prim_path = common.import_rig()
    print(f"[katzlab] imported placeholder rig at {prim_path}")

    world = World(stage_units_in_meters=1.0)
    stage = world.stage
    world.scene.add_default_ground_plane()
    articulation = Articulation(prim_paths_expr=prim_path, name="ureteroscope")
    world.scene.add(articulation)
    world.reset()
    print(f"[katzlab] articulation joints: {list(articulation.dof_names)}")

    camera_mount_path = common.find_camera_mount_path(stage, prim_path)
    common.create_camera(stage, camera_mount_path)
    print(f"[katzlab] camera created at {camera_mount_path}/Camera")
    print(
        "[katzlab] select it in the viewport's camera dropdown to see what "
        "the scope sees"
    )

    anatomy_path = common.build_placeholder_anatomy(stage)
    print(f"[katzlab] placeholder anatomy at {anatomy_path}")

    _scatter_static_stones(stage, common, count=args.num_stones)
    print(f"[katzlab] scattered {args.num_stones} static placeholder stones")

    print("[katzlab] scene ready -- window open, close it or Ctrl+C when done looking")
    _spin(simulation_app, world)


def _scatter_static_stones(stage, common, count: int) -> None:
    """Places a few stones once, no physics, no randomization loop -- just
    enough to eyeball scale and placement against the anatomy."""
    from pxr import UsdGeom, UsdShade

    root_path = "/World/stones"
    UsdGeom.Xform.Define(stage, root_path)
    material = common.make_material(stage, root_path + "/material", common.STONE_COLOR_RANGE[0])

    for i in range(count):
        stone_path = f"{root_path}/stone_{i}"
        stone = UsdGeom.Sphere.Define(stage, stone_path)
        radius = random.uniform(*common.STONE_SCALE_RANGE_M)
        stone.GetRadiusAttr().Set(radius)
        position = (
            random.uniform(0.01, common.ANATOMY_LENGTH_M - 0.01),
            random.uniform(-0.002, 0.002),
            random.uniform(-0.002, 0.002),
        )
        UsdGeom.XformCommonAPI(stone).SetTranslate(position)
        UsdShade.MaterialBindingAPI(stone).Bind(material)
        common.tag_semantic(stage, stone_path, "stone")


def _spin(simulation_app, world) -> None:
    try:
        while simulation_app.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    try:
        main()
    except Exception:
        raise
    sys.exit(0)
