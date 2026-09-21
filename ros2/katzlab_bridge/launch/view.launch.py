"""robot_state_publisher + RViz, fed by /ureteroscope/joint_states.

Does NOT start the bridge node itself -- pair it with a separately running
`ros2 launch katzlab_bridge bridge.launch.py`, or use sim.launch.py to get
both at once. Kept separate so RViz can be pointed at a bag or at Isaac Sim's
own joint_states republish just as easily as at the live rig.

    ros2 launch katzlab_bridge view.launch.py
"""

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node

URDF_PATH = Path(__file__).resolve().parent.parent / "urdf" / "ureteroscope.urdf"
RVIZ_CONFIG = Path(__file__).resolve().parent.parent / "rviz" / "ureteroscope.rviz"


def generate_launch_description() -> LaunchDescription:
    robot_description = URDF_PATH.read_text()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        remappings=[("/joint_states", "/ureteroscope/joint_states")],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(RVIZ_CONFIG)],
    )

    return LaunchDescription([robot_state_publisher, rviz])
