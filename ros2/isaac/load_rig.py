"""Load the ureteroscope rig into Isaac Sim and drive it from ROS 2.

Standalone Isaac Sim script (not a ROS node -- Isaac Sim is its own
application). Run it with Isaac Sim's own Python, after sourcing ROS first
(see ros2/README.md section 2 for why sourcing order matters):

    source /opt/ros/jazzy/setup.bash
    ~/isaacsim/python.sh ros2/isaac/load_rig.py

    # or, if isaac-sim.sh is what your install exposes on PATH:
    isaac-sim.sh --exec "$(pwd)/ros2/isaac/load_rig.py"

WHAT THIS DOES
  1. Boots Isaac Sim headed (a window opens) with an empty stage.
  2. Imports urdf/ureteroscope.urdf (the same file RViz uses) via the URDF
     importer extension, so the digital twin is the same kinematic tree,
     not a second copy of it hand-built in USD.
  3. Subscribes to /ureteroscope/joint_states over rclpy and mirrors every
     sample onto the imported articulation's joint positions.

DIRECTION: rig -> sim only. This script never publishes cmd_velocity or
anything else back onto the ROS graph -- it is a passive mirror, the same
"motion over ROS, laser never" caution the bridge node itself applies, just
pointed the other way: a simulator should not be able to move the real rig
by mistake either. Driving the physical rig from a simulated plan is a real
and reasonable thing to eventually want, but it should be a deliberate,
separately-reviewed path (e.g. a script that publishes to
/ureteroscope/cmd_velocity the same way any other ROS client would, going
through the bridge's normal safety plumbing), not something this mirror
script does implicitly.

UNVERIFIED: written against NVIDIA's documented Isaac Sim 6.x / ROS 2
Jazzy APIs (see ros2/README.md section 2) but not run against a real Isaac
Sim install -- there was none available to test against here. The two
likeliest breakage points on a real machine, in order:
  - The URDF importer extension's Python module path. Isaac Sim 6.x moved
    it to `isaacsim.asset.importer.urdf`; older installs (5.x and before)
    use `omni.importer.urdf`. This script tries the new path first and
    falls back to the old one.
  - The exact import-config field names on `ImportConfig` -- these have
    changed across Isaac Sim releases. If `import_urdf()` raises an
    AttributeError on a config field, check that extension's own example
    scripts (Isaac Sim ships them under
    extension_examples/interactive_scripts) for the current field names
    and adjust URDF_IMPORT_CONFIG_OVERRIDES in _isaac_common.py rather than
    the rest of the script.

The URDF import plumbing shared with generate_training_data.py lives in
_isaac_common.py.
"""

from __future__ import annotations

import sys
import threading

JOINT_STATES_TOPIC = "/ureteroscope/joint_states"

# ---------------------------------------------------------------------------
# 1. Boot Isaac Sim. Must happen before any other isaacsim/omni import --
#    including _isaac_common, which is why this import sits below the boot
#    line rather than at the top of the file with everything else.
# ---------------------------------------------------------------------------
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

from _isaac_common import import_rig  # noqa: E402


class JointMirrorNode(Node):
    """Subscribes to /ureteroscope/joint_states and stores the latest sample.

    Kept deliberately dumb: it does not touch the articulation itself,
    because that has to happen on Isaac Sim's own render thread via
    ``world.step()``, not from the rclpy executor thread. See ``main()``.
    """

    def __init__(self, joint_names: list[str]) -> None:
        super().__init__("katzlab_isaac_mirror")
        self.joint_names = joint_names
        self.latest: dict[str, float] = {}
        self._lock = threading.Lock()
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joint_states, 10)

    def _on_joint_states(self, msg: JointState) -> None:
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                self.latest[name] = position

    def snapshot(self) -> dict[str, float]:
        with self._lock:
            return dict(self.latest)


def main() -> None:
    prim_path = import_rig()
    print(f"[katzlab] imported rig at {prim_path}")

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    articulation = Articulation(prim_paths_expr=prim_path, name="ureteroscope")
    world.scene.add(articulation)
    world.reset()

    joint_names = list(articulation.dof_names)
    print(f"[katzlab] articulation joints: {joint_names}")

    if not rclpy.ok():
        rclpy.init()
    ros_node = JointMirrorNode(joint_names)

    # rclpy's own executor runs in a background thread; Isaac Sim's render
    # loop stays on the main thread, which is the one that is actually
    # allowed to touch the stage/physics state.
    ros_thread = threading.Thread(
        target=rclpy.spin, args=(ros_node,), daemon=True
    )
    ros_thread.start()

    try:
        while simulation_app.is_running():
            latest = ros_node.snapshot()
            if latest:
                positions = [latest.get(name, 0.0) for name in joint_names]
                articulation.set_joint_positions(positions)
            world.step(render=True)
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        simulation_app.close()
        raise
    sys.exit(0)
