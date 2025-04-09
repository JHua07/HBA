import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import launch


def generate_launch_description():
    config_path = PathJoinSubstitution([get_package_share_directory("hba"), "config", "hba.yaml"])

    return launch.LaunchDescription(
        [
            LogInfo(msg=[config_path]),
            Node(
                package="hba",
                namespace="hba",
                executable="hba",
                name="hba_launch",
                output="screen",
                parameters=[
                    config_path
                ],
            )
        ]
    )
