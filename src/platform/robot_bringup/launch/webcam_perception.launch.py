from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def package_file(package, *parts):
    return PathJoinSubstitution([FindPackageShare(package), *parts])


def generate_launch_description():
    camera_params = LaunchConfiguration("camera_params")
    preproc_params = LaunchConfiguration("preproc_params")
    detector_params = LaunchConfiguration("detector_params")
    calibration_params = LaunchConfiguration("calibration_params")
    health_params = LaunchConfiguration("health_params")
    latency_params = LaunchConfiguration("latency_params")

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_params",
            default_value=package_file("sensor_drivers", "config", "usb_webcam.yaml"),
        ),
        DeclareLaunchArgument(
            "preproc_params",
            default_value=package_file("perception_preproc", "config", "yolo_preproc.yaml"),
        ),
        DeclareLaunchArgument(
            "detector_params",
            default_value=package_file("perception_inference", "config", "debug_detector.yaml"),
        ),
        DeclareLaunchArgument(
            "calibration_params",
            default_value=package_file("tf_and_calibration", "config", "calibration_validator.yaml"),
        ),
        DeclareLaunchArgument(
            "health_params",
            default_value=package_file("logging_and_diagnostics", "config", "health_aggregator.yaml"),
        ),
        DeclareLaunchArgument(
            "latency_params",
            default_value=package_file("benchmarks", "config", "latency_probe.yaml"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("tf_and_calibration", "launch", "static_tf.launch.py")),
        ),
        Node(
            package="sensor_drivers",
            executable="usb_camera_node",
            name="usb_camera_node",
            output="screen",
            parameters=[camera_params],
            remappings=[
                ("image_raw", "/sensors/rgb/image_raw"),
                ("camera_info", "/sensors/rgb/camera_info"),
                ("health", "/diagnostics/usb_camera/health"),
            ],
        ),
        Node(
            package="perception_preproc",
            executable="preproc_node",
            name="preproc_node",
            output="screen",
            parameters=[preproc_params],
        ),
        Node(
            package="perception_inference",
            executable="detector_node",
            name="detector_node",
            output="screen",
            parameters=[detector_params],
        ),
        Node(
            package="tf_and_calibration",
            executable="calibration_validator_node",
            name="calibration_validator_node",
            output="screen",
            parameters=[calibration_params],
        ),
        Node(
            package="logging_and_diagnostics",
            executable="health_aggregator_node",
            name="health_aggregator_node",
            output="screen",
            parameters=[health_params],
        ),
        Node(
            package="benchmarks",
            executable="latency_probe_node",
            name="latency_probe_node",
            output="screen",
            parameters=[latency_params],
        ),
    ])
