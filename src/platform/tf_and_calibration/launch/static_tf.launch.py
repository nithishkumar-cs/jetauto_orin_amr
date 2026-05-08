from launch import LaunchDescription
from launch_ros.actions import Node


def static_tf(name, parent, child, xyz, rpy):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x", str(xyz[0]),
            "--y", str(xyz[1]),
            "--z", str(xyz[2]),
            "--roll", str(rpy[0]),
            "--pitch", str(rpy[1]),
            "--yaw", str(rpy[2]),
            "--frame-id", parent,
            "--child-frame-id", child,
        ],
    )


def generate_launch_description():
    return LaunchDescription([
        static_tf(
            "base_to_rgb_camera_link",
            "base_link",
            "rgb_camera_link",
            [0.18, 0.0, 0.28],
            [0.0, 0.0, 0.0],
        ),
        static_tf(
            "rgb_camera_link_to_optical",
            "rgb_camera_link",
            "rgb_camera_optical_frame",
            [0.0, 0.0, 0.0],
            [-1.57079632679, 0.0, -1.57079632679],
        ),
        static_tf(
            "base_to_depth_camera_link",
            "base_link",
            "depth_camera_link",
            [0.20, 0.0, 0.30],
            [0.0, 0.0, 0.0],
        ),
        static_tf(
            "base_to_lidar_link",
            "base_link",
            "lidar_link",
            [0.12, 0.0, 0.18],
            [0.0, 0.0, 0.0],
        ),
    ])
