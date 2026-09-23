"""Send one navigate task attempt, with the metric/constraint params loaded
from config/navigate_metrics.yaml and the goal pose given as launch arguments.

    ros2 launch rtr_navigate navigate_client.launch.py x:=2.0 y:=1.0 theta:=1.57
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_params_file = str(Path(get_package_share_directory("rtr_navigate")) / "config" / "navigate_metrics.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("x", description="Target x in the odom frame."),
            DeclareLaunchArgument("y", description="Target y in the odom frame."),
            DeclareLaunchArgument("theta", default_value="0.0", description="Target heading in radians."),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="YAML file setting the metric/constraint parameters.",
            ),
            Node(
                package="rtr_navigate",
                executable="client",
                output="screen",
                arguments=[LaunchConfiguration("x"), LaunchConfiguration("y"), LaunchConfiguration("theta")],
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
