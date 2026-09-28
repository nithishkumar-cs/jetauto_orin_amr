from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_ekf(context):
    backend = LaunchConfiguration("backend").perform(context)
    if backend not in ("hardware", "isaac", "rosbag"):
        raise RuntimeError(f"Unsupported odometry backend: {backend}")

    profile = "hardware" if backend == "hardware" else "isaac"
    config_path = (
        Path(get_package_share_directory("localization_odometry_fusion"))
        / "config"
        / f"ekf_{profile}.yaml"
    )
    return [
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="odometry_fusion",
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
            OpaqueFunction(function=launch_ekf),
        ]
    )
