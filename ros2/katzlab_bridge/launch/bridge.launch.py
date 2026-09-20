"""Launch the katzlab <-> ROS 2 bridge.

    ros2 launch katzlab_bridge bridge.launch.py
    ros2 launch katzlab_bridge bridge.launch.py dry_run:=true   # no Teensy needed
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value="",
        description="Path to system.yaml (empty = katzlab's own default)",
    )
    dry_run_arg = DeclareLaunchArgument(
        "dry_run",
        default_value="false",
        description="Run without a Teensy attached (exercises ROS wiring only)",
    )

    node = Node(
        package="katzlab_bridge",
        executable="bridge_node",
        name="katzlab_bridge",
        output="screen",
        parameters=[
            {
                "config_path": LaunchConfiguration("config_path"),
                "dry_run": LaunchConfiguration("dry_run"),
            }
        ],
    )

    return LaunchDescription([config_path_arg, dry_run_arg, node])
