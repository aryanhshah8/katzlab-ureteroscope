"""Bridge + Foxglove's WebSocket bridge, so anything on the network -- a Mac
with no ROS 2 install, in particular -- can watch the whole graph.

    ros2 launch katzlab_bridge watch.launch.py dry_run:=true

Then, from the Mac: open Foxglove Studio, "Open connection" -> Foxglove
WebSocket -> ws://<mint-hostname-or-ip>:8765. No ROS 2 install on the Mac
side at all -- see ros2/README.md section 6 for what to check once
connected.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    return LaunchDescription([config_path_arg, dry_run_arg, port_arg, bridge, foxglove_bridge])
