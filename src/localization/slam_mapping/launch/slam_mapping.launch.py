from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_mapping(context):
    backend = LaunchConfiguration("backend").perform(context)
    if backend not in ("hardware", "isaac", "rosbag"):
        raise RuntimeError(f"Unsupported SLAM backend: {backend}")

    config_path = (
        Path(get_package_share_directory("localization_slam_mapping"))
        / "config"
        / "slam_mapping.yaml"
    )
    return [
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[str(config_path), {"use_sim_time": backend != "hardware"}],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="isaac"),
            DeclareLaunchArgument("log_level", default_value="info"),
            OpaqueFunction(function=launch_mapping),
        ]
    )
