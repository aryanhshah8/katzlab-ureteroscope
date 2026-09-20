"""The ROS 2 <-> katzlab bridge node.

Wraps the existing control stack -- it owns the serial link, the safety
interlocks and the tuning, all unchanged -- and exposes it to the ROS graph:

    publish    /ureteroscope/joint_states   sensor_msgs/JointState
    publish    /ureteroscope/laser_state    diagnostic_msgs/DiagnosticStatus
    subscribe  /ureteroscope/cmd_velocity   geometry_msgs/Twist
    service    /ureteroscope/home           std_srvs/Trigger
    service    /ureteroscope/estop          std_srvs/Trigger

Units: ROS is SI, the rig's own config and firmware are mm and degrees. This
node converts at the boundary; nothing about the underlying tuning changes.

``cmd_velocity`` repurposes Twist's unused fields rather than introducing a
new message type this early: ``linear.x`` is linear velocity (m/s),
``angular.z`` is rotation velocity (rad/s), ``angular.y`` is flexion velocity
(rad/s). If this grows a real client base, replace it with a dedicated
message -- Twist was picked to avoid standing up a whole interface package
for three floats.

The laser is intentionally absent from the topic/service list above. See
``ros2/README.md`` and ``ros_input_source.py``: nothing about the laser is
ever commandable from ROS.
"""

from __future__ import annotations

import math
import sys
import threading
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

# ``katzlab`` lives in ../../python relative to this file, as a plain
# (non-ROS) Python package. Rather than packaging/installing it separately --
# which would mean two copies of the same code to keep in sync -- this node
# just puts it on sys.path at import time. It is still the one real katzlab
# package on disk; nothing here forks or reimplements it.
_REPO_ROOT = Path(__file__).resolve().parents[3]
_PYTHON_SRC = _REPO_ROOT / "python"
if str(_PYTHON_SRC) not in sys.path:
    sys.path.insert(0, str(_PYTHON_SRC))

from katzlab.cli import make_motion_driver, open_links  # noqa: E402
from katzlab.config import SystemConfig, load_system_config  # noqa: E402
from katzlab.control import ControlLoop  # noqa: E402
from katzlab.drivers import LaserDriver, MotionDriver  # noqa: E402

from .ros_input_source import Ros2InputSource  # noqa: E402

JOINT_NAMES = ["linear", "rotation", "flexion"]


def _mm_to_m(mm: float) -> float:
    return mm / 1000.0


def _deg_to_rad(deg: float) -> float:
    return math.radians(deg)


class BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("katzlab_bridge")

        self.declare_parameter("config_path", "")
        self.declare_parameter("dry_run", False)
        self.declare_parameter("publish_hz", 20.0)

        config_path = self.get_parameter("config_path").value or None
        dry_run = bool(self.get_parameter("dry_run").value)

        self._cfg: SystemConfig = load_system_config(config_path)
        self._source = Ros2InputSource()

        self._motion_link = None
        self._laser_link = None
        motion: MotionDriver
        laser: LaserDriver | None

        if dry_run:
            # Dry-run always uses the chunked driver's dry-run shim -- the
            # velocity driver (the protocol actually run on hardware) talks
            # to a real link with no simulated fallback. This path exists to
            # exercise the ROS wiring (topics, services, message shapes)
            # without a Teensy attached, not to stand in physically for
            # Isaac Sim.
            self.get_logger().warn(
                "dry_run: using the chunked driver's no-hardware shim, "
                "not the velocity protocol actually run on the rig"
            )
            motion = MotionDriver(None, self._cfg.motion, dry_run=True)
            laser = LaserDriver(None, self._cfg.laser, dry_run=True)
        else:
            self._motion_link, self._laser_link = open_links(self._cfg)
            motion = make_motion_driver(self._motion_link, self._cfg)
            motion.sync(timeout_s=25.0)
            laser = LaserDriver(self._laser_link, self._cfg.laser)

        self._motion = motion
        self._laser = laser
        self._loop = ControlLoop(self._cfg, self._source, motion, laser)

        self._joint_pub = self.create_publisher(JointState, "/ureteroscope/joint_states", 10)
        self._laser_pub = self.create_publisher(
            DiagnosticStatus, "/ureteroscope/laser_state", 10
        )
        self.create_subscription(Twist, "/ureteroscope/cmd_velocity", self._on_cmd_velocity, 10)
        self.create_service(Trigger, "/ureteroscope/home", self._on_home)
        self.create_service(Trigger, "/ureteroscope/estop", self._on_estop)

        self._loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._loop_thread.start()

        self.get_logger().info(
            f"katzlab_bridge up (protocol={self._cfg.motion.protocol}, dry_run={dry_run})"
        )

    # -- ROS callbacks -------------------------------------------------

    def _on_cmd_velocity(self, msg: Twist) -> None:
        motion = self._cfg.motion
        linear_axis = _to_axis(msg.linear.x, _mm_to_m(motion.linear.max_rate_mm_s))
        rotation_axis = _to_axis(msg.angular.z, _deg_to_rad(motion.rotation.max_rate_deg_s))
        flexion_axis = _to_axis(msg.angular.y, _deg_to_rad(motion.flexion.max_rate_deg_s))
        self._source.set_axes(linear=linear_axis, rotation=rotation_axis, flexion=flexion_axis)

    def _on_home(self, request, response):  # noqa: ARG002
        self._source.request_home()
        response.success = True
        response.message = "home requested"
        return response

    def _on_estop(self, request, response):  # noqa: ARG002
        self._source.request_estop()
        response.success = True
        response.message = "estop requested"
        return response

    # -- control loop ---------------------------------------------------

    def _run_loop(self) -> None:
        self._loop.run(status_callback=self._publish_status)

    def _publish_status(self, status: dict) -> None:
        now = self.get_clock().now().to_msg()
        motion_state = self._motion.state

        joints = JointState()
        joints.header.stamp = now
        joints.name = JOINT_NAMES
        joints.position = [
            _mm_to_m(motion_state.linear_mm),
            _deg_to_rad(motion_state.rotation_map_deg),
            _deg_to_rad(motion_state.flexion_tip_deg),
        ]
        self._joint_pub.publish(joints)

        if self._laser is not None:
            laser_state = self._laser.state
            diag = DiagnosticStatus()
            diag.name = "ureteroscope_laser"
            diag.level = DiagnosticStatus.WARN if laser_state.firing else DiagnosticStatus.OK
            diag.message = "firing" if laser_state.firing else (
                "armed" if laser_state.armed else "disarmed"
            )
            diag.values = [
                KeyValue(key="enabled", value=str(laser_state.enabled)),
                KeyValue(key="ready", value=str(laser_state.ready)),
                KeyValue(key="firing", value=str(laser_state.firing)),
                KeyValue(key="estopped", value=str(status.get("estopped", False))),
            ]
            self._laser_pub.publish(diag)

    def destroy_node(self) -> bool:
        self._loop.stop()
        self._loop_thread.join(timeout=5.0)
        for link in {self._motion_link, self._laser_link}:
            if link is not None:
                link.close()
        return super().destroy_node()


def _to_axis(value: float, max_value: float) -> float:
    """SI target -> normalised -1..1 axis, clamped. ``max_value`` in the same
    units as ``value`` (both m/s, or both rad/s)."""
    if max_value <= 0.0:
        return 0.0
    return max(-1.0, min(1.0, value / max_value))


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
