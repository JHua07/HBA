import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import launch_ros
from launch_ros.actions import Node
import launch


def generate_launch_description():
    config_path = PathJoinSubstitution([get_package_share_directory("hba"), "config", "visualize.yaml"])
    rviz_cfg = PathJoinSubstitution([get_package_share_directory("hba"), "rviz_cfg", "visualize.rviz"])

    return launch.LaunchDescription(
        [
            LogInfo(msg=[config_path]),
            launch_ros.actions.Node(
                package="hba",
                namespace="visualize",
                executable="visualize",
                name="visualize_launch",
                output="screen",
                parameters=[
                    config_path
                ]
            ),
            launch_ros.actions.Node(
                package="rviz2",
                namespace="visualize",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
            )
        ]
    )
