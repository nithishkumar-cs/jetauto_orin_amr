from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam_params = LaunchConfiguration("slam_params")

    return LaunchDescription([
        DeclareLaunchArgument(
            "slam_params",
            default_value=PathJoinSubstitution([
                FindPackageShare("localization_mapping"),
                "config",
                "slam_toolbox_async.yaml",
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            ])),
            launch_arguments={"slam_params_file": slam_params}.items(),
        ),
    ])

