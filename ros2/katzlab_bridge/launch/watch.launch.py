"""Bridge + Foxglove's WebSocket bridge + robot_state_publisher, so a Mac
with no ROS 2 install can watch the whole graph AND see a live 3D model of
the rig moving, entirely inside Foxglove Studio's own 3D panel.

    ros2 launch katzlab_bridge watch.launch.py dry_run:=true

Then, from the Mac: open Foxglove Studio, "Open connection" -> Foxglove
WebSocket -> ws://<mint-hostname-or-ip>:8765, then add a "3D" panel --
Foxglove reads /robot_description + /tf (both published here) and draws
the URDF moving live, the same idea as RViz but with no ROS 2 install and
no GUI access to the Mint box needed at all. See ros2/README.md section 6.

IMPORTANT: this always shows the PLACEHOLDER box/cylinder URDF
(../urdf/ureteroscope.urdf), never the real rig CAD
(../../isaac/assets/glidar_robot/) -- Foxglove's 3D panel reads URDF, and
only the placeholder exists as URDF; the real rig is USD, Isaac-Sim-only.
To see the real CAD (Isaac Sim's own render), there is no Mac-only path --
Isaac Sim itself cannot run on macOS, so that means either being at the
Mint box's own screen or remote-desktop/VNC into it. Foxglove can only
ever show what's published as ROS 2 topics, and nothing publishes the real
mesh as one.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

URDF_PATH = Path(__file__).resolve().parent.parent / "urdf" / "ureteroscope.urdf"


def generate_launch_description() -> LaunchDescription:
    config_path_arg = DeclareLaunchArgument("config_path", default_value="")
    dry_run_arg = DeclareLaunchArgument("dry_run", default_value="false")
    port_arg = DeclareLaunchArgument("port", default_value="8765")

    pkg_share = FindPackageShare("katzlab_bridge")

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "bridge.launch.py"])
        ),
        launch_arguments={
            "config_path": LaunchConfiguration("config_path"),
            "dry_run": LaunchConfiguration("dry_run"),
        }.items(),
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{"port": LaunchConfiguration("port")}],
    )

    # Same purpose as view.launch.py's robot_state_publisher node, minus
    # rviz -- a GUI app on Mint does nothing for a Mac-only viewer, and
    # Foxglove's own 3D panel is the thing actually consuming this.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": URDF_PATH.read_text()}],
        remappings=[("/joint_states", "/ureteroscope/joint_states")],
    )

    return LaunchDescription(
        [config_path_arg, dry_run_arg, port_arg, bridge, foxglove_bridge, robot_state_publisher]
    )
