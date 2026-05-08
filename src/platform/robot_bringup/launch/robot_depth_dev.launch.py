from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def package_file(package, *parts):
    return PathJoinSubstitution([FindPackageShare(package), *parts])


def generate_launch_description():
    detector_params = LaunchConfiguration("detector_params")

    return LaunchDescription([
        DeclareLaunchArgument(
            "detector_params",
            default_value=package_file("perception_inference", "config", "yolo_detector.yaml"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("tf_and_calibration", "launch", "static_tf.launch.py")),
        ),
        Node(
            package="base_interface",
            executable="jetauto_base_interface_node",
            name="jetauto_base_interface_node",
            output="screen",
            parameters=[package_file("base_interface", "config", "base_interface.yaml")],
        ),
        Node(
            package="perception_inference",
            executable="detector_node",
            name="detector_node",
            output="screen",
            parameters=[detector_params],
        ),
        Node(
            package="perception_geometry",
            executable="depth_obstacle_projector_node",
            name="depth_obstacle_projector_node",
            output="screen",
            parameters=[package_file("perception_geometry", "config", "depth_projection.yaml")],
        ),
        Node(
            package="tracking_fusion",
            executable="obstacle_tracker_node",
            name="obstacle_tracker_node",
            output="screen",
            parameters=[package_file("tracking_fusion", "config", "tracking.yaml")],
        ),
        Node(
            package="sensor_fusion",
            executable="obstacle_fusion_node",
            name="obstacle_fusion_node",
            output="screen",
            parameters=[package_file("sensor_fusion", "config", "fusion.yaml")],
        ),
        Node(
            package="safety_layer",
            executable="safety_monitor_node",
            name="safety_monitor_node",
            output="screen",
            parameters=[package_file("safety_layer", "config", "safety.yaml")],
        ),
        Node(
            package="logging_and_diagnostics",
            executable="health_aggregator_node",
            name="health_aggregator_node",
            output="screen",
            parameters=[package_file("logging_and_diagnostics", "config", "health_aggregator.yaml")],
        ),
        Node(
            package="benchmarks",
            executable="latency_probe_node",
            name="latency_probe_node",
            output="screen",
            parameters=[package_file("benchmarks", "config", "latency_probe.yaml")],
        ),
    ])

