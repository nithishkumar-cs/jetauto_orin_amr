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
                FindPackageShare("sensor_drivers"),
                "config",
                "rgb_camera.yaml",
            ]),
        ),
        Node(
            package="sensor_drivers",
            executable="usb_camera_node",
            name="rgb_camera_node",
            output="screen",
            parameters=[params],
            remappings=[
                ("image_raw", "/sensors/rgb/image_raw"),
                ("camera_info", "/sensors/rgb/camera_info"),
                ("health", "/diagnostics/rgb_camera/health"),
            ],
        ),
    ])
