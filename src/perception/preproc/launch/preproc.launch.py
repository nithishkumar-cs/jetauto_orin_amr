from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = LaunchConfiguration("params")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params",
            default_value=PathJoinSubstitution([
                FindPackageShare("perception_preproc"),
                "config",
                "yolo_preproc.yaml",
            ]),
        ),
        Node(
            package="perception_preproc",
            executable="preproc_node",
            name="preproc_node",
            output="screen",
            parameters=[params],
        ),
    ])

