from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("se3_hopf"), "config", "default.yaml"]
                ),
            ),
            Node(
                package="se3_hopf",
                executable="se3_hopf_node",
                name="se3_hopf_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
