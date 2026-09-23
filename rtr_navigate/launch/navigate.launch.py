"""Bring up the simulated robot, the navigate task, and RViz.

Send a goal once this is running:

    ros2 run rtr_navigate client 2.0 1.0 1.57
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("rtr_navigate"))
    urdf = (share / "urdf" / "diffbot.urdf").read_text()
    rviz_config = str(share / "rviz" / "navigate.rviz")

    use_rviz = LaunchConfiguration("rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true", description="Start RViz."),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": urdf}],
            ),
            Node(
                package="rtr_navigate",
                executable="sim",
                name="diff_drive_sim",
                output="screen",
            ),
            # No name= here: this process hosts two nodes (navigate_executor and
            # navigate_atomic_task), and a name remap would collide them.
            Node(
                package="rtr_navigate",
                executable="navigate",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
