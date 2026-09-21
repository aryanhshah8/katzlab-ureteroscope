"""Everything at once: the bridge node + robot_state_publisher + RViz.

    ros2 launch katzlab_bridge sim.launch.py                  # real Teensy
    ros2 launch katzlab_bridge sim.launch.py dry_run:=true    # no Teensy

Isaac Sim is launched separately (it is its own application, not a ROS
node) -- see ros2/isaac/README.md. Once it is up with its ROS 2 Bridge
extension enabled and subscribed to /ureteroscope/joint_states, it mirrors
whatever RViz is showing here; nothing further to launch for that.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_path_arg = DeclareLaunchArgument("config_path", default_value="")
    dry_run_arg = DeclareLaunchArgument("dry_run", default_value="false")

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

    view = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "view.launch.py"])
        )
    )

    return LaunchDescription([config_path_arg, dry_run_arg, bridge, view])
