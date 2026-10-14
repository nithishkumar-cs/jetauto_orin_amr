#!/usr/bin/env python3
"""Publish empty perception outputs or zero commands for disabled debug stages."""

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from vision_msgs.msg import Detection2DArray, Detection3DArray
from amr_interfaces.msg import TrackedObstacleArray
from geometry_msgs.msg import Twist


# Each entry is input type/topic, output type/topic, and output frame override.
# An empty result is tied to the source stamp so downstream synchronizers can
# distinguish a real zero-observation cycle from an absent input source.
CONTRACTS = {
    "detector": [(Image, "/camera/rgb/image_rect_color", Detection2DArray,
                  "/perception/detections_2d", None)],
    "geometry": [(Detection2DArray, "/perception/detections_2d", Detection3DArray,
                  "/perception/detections_3d", None)],
    "tracking": [
        (Detection3DArray, "/perception/detections_3d", TrackedObstacleArray,
         "/perception/camera/tracks", "base_link"),
        (Detection3DArray, "/perception/lidar/clusters", TrackedObstacleArray,
         "/perception/lidar/tracks", "base_link"),
    ],
    "fusion": [
        (TrackedObstacleArray, "/perception/camera/tracks", TrackedObstacleArray,
         "/perception/fused_obstacles", "base_link"),
        (TrackedObstacleArray, "/perception/lidar/tracks", TrackedObstacleArray,
         "/perception/fused_obstacles", "base_link"),
    ],
    "lidar_clustering": [(LaserScan, "/scan", Detection3DArray,
                          "/perception/lidar/clusters", None)],
}


def empty_output(source, message_type, frame):
    """Create an empty array with a copied source header and chosen frame."""
    result = message_type()
    result.header = deepcopy(source.header)
    if frame is not None:
        result.header.frame_id = frame
    return result


class SyntheticSource(Node):
    """Relay source stamps as empty arrays or publish a stopped command."""

    def __init__(self):
        """Create the selected synthetic output; reject unknown components."""
        super().__init__("synthetic_source")
        component = self.declare_parameter("component", "").value
        if component == "safety":
            self.stop_publisher = self.create_publisher(Twist, "/cmd_vel/safety_limited", 10)
            self.create_timer(0.05, self._publish_stop)
            return
        if component not in CONTRACTS:
            raise ValueError(f"No synthetic output contract for {component!r}")
        for input_type, input_topic, output_type, output_topic, frame in CONTRACTS[component]:
            publisher = self.create_publisher(output_type, output_topic, 10)

            def relay(source, *, message_type=output_type, target=publisher, output_frame=frame):
                """Publish an empty array with the source timestamp and valid frame."""
                target.publish(empty_output(source, message_type, output_frame))

            input_qos = (
                qos_profile_sensor_data
                if input_type in (Image, LaserScan, Detection2DArray)
                else 10
            )
            self.create_subscription(input_type, input_topic, relay, input_qos)

    def _publish_stop(self):
        """Publish zero velocity while the safety chain is disabled."""
        self.stop_publisher.publish(Twist())


def main(args=None):
    """Spin the synthetic source until launch shuts it down."""
    rclpy.init(args=args)
    node = None
    try:
        node = SyntheticSource()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
