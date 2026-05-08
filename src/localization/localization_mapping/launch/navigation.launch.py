from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("localization_mapping"),
                "config",
                "nav2_params.yaml",
            ]),
        ),
        DeclareLaunchArgument("map", default_value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            ])),
            launch_arguments={
                "params_file": params_file,
                "map": map_file,
                "use_sim_time": "false",
            }.items(),
        ),
    ])

