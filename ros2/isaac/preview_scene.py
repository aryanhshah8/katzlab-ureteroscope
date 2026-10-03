"""Build the CV training scene and just sit there, so you can look at it
before committing to a bulk capture run with generate_training_data.py.

    ~/isaacsim/python.sh ros2/isaac/preview_scene.py

Opens a window with: the imported rig, the placeholder anatomy (a ureter
tube opening into a calyx chamber), a handful of statically-placed stones,
and the camera mount visible at the flexion tip. Nothing is randomized or
captured -- this is the setup step, not the data-generation step. It uses
the exact same scene-building code as generate_training_data.py
(_isaac_common.py), so what you're looking at here is what that script will
actually render from, not a separate approximation of it.

WHAT TO CHECK BEFORE MOVING ON:
  1. Select the scope's own camera in the viewport (Isaac Sim's viewport
     camera dropdown -- the prim path is printed to the console on startup)
     and confirm it's actually looking down the lumen, not at a wall or
     backward out the tube. This is the single most likely thing to be
     wrong, since the mount offset/orientation in the URDF is a guess (see
     CAMERA-SPECS.md).
  2. Confirm the anatomy's inner wall is visible and shaded, not black/
     missing -- a missing double-sided flag reads as the tube appearing
     hollow/invisible from inside.
  3. Confirm stones are inside the lumen, not floating outside it or
     clipped through a wall.
  4. Use the joint sliders (or drive the articulation manually) through
     their full range and re-check #1 -- the camera moves with flexion_link,
     so a mount that looks fine at rest can point somewhere wrong once the
     tip bends.

Once all four look right, move on to generate_training_data.py.

UNVERIFIED, same caveat as the rest of ros2/isaac: written against NVIDIA's
documented Isaac Sim 6.x API, not run against a real install.
"""

from __future__ import annotations

import argparse
import random
import sys


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--num-stones", type=int, default=3, help="How many placeholder stones to scatter."
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
        _run(args, simulation_app, World, Articulation, common)
    finally:
        simulation_app.close()


def _run(args, simulation_app, World, Articulation, common) -> None:
    prim_path = common.import_rig()
    print(f"[katzlab] imported rig at {prim_path}")

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
        "the scope sees -- this is check #1 in the module docstring"
    )

    anatomy_path = common.build_placeholder_anatomy(stage)
    print(f"[katzlab] placeholder anatomy at {anatomy_path}")

    _scatter_static_stones(stage, common, count=args.num_stones)
    print(f"[katzlab] scattered {args.num_stones} static placeholder stones")

    print("[katzlab] scene ready -- window open, close it or Ctrl+C when done looking")
    try:
        while simulation_app.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        pass


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


if __name__ == "__main__":
    try:
        main()
    except Exception:
        raise
    sys.exit(0)
