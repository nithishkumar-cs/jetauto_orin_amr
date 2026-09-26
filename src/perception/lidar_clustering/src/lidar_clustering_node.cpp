#include "perception_lidar_clustering/lidar_clustering_node.hpp"

#include <functional>
#include <stdexcept>
#include <string>

namespace perception_lidar_clustering
{

LidarClusteringNode::LidarClusteringNode(const rclcpp::NodeOptions & options)
: Node("lidar_clustering", options), clusterer_()
{
  auto config = LidarClustererConfig{};
  config.max_adjacent_point_distance_m = declare_parameter<double>(
    "max_adjacent_point_distance_m", config.max_adjacent_point_distance_m);
  const auto min_cluster_points =
    declare_parameter<int>("min_cluster_points", static_cast<int>(config.min_cluster_points));
  if (min_cluster_points <= 0) {
    throw std::invalid_argument("min_cluster_points must be positive.");
  }
  config.min_cluster_points = static_cast<std::size_t>(min_cluster_points);
  clusterer_ = LidarClusterer(config);

  const auto scan_in_topic = declare_parameter<std::string>("scan_in_topic", "/scan");
  const auto clusters_out_topic =
    declare_parameter<std::string>("clusters_out_topic", "/perception/lidar/clusters");
  const auto scan_queue_size = declare_parameter<int>("scan_queue_size", 5);
  if (scan_queue_size <= 0) {
    throw std::invalid_argument("scan_queue_size must be positive.");
  }

  clusters_publisher_ =
    create_publisher<vision_msgs::msg::Detection3DArray>(clusters_out_topic, rclcpp::QoS(10));
  auto scan_qos = rclcpp::SensorDataQoS();
  scan_qos.keep_last(static_cast<std::size_t>(scan_queue_size));
  subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_in_topic, scan_qos, std::bind(&LidarClusteringNode::on_scan, this, std::placeholders::_1));
}

void LidarClusteringNode::on_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr & message)
{
  if (message->header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "LaserScan frame_id is empty; scan was not clustered.");
    return;
  }

  const auto scan = LaserScanView::create(
    message->ranges.size(), message->angle_min, message->angle_max, message->angle_increment,
    message->range_min, message->range_max, message->ranges.data());
  if (!scan) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "LaserScan metadata or range storage is invalid; scan was not clustered.");
    return;
  }

  vision_msgs::msg::Detection3DArray output;
  output.header = message->header;
  const auto clusters = clusterer_.cluster(*scan);
  output.detections.reserve(clusters.size());
  for (const auto & cluster : clusters) {
    auto & detection = output.detections.emplace_back();
    detection.header = output.header;
    detection.bbox.center.position.x = cluster.center_x_m;
    detection.bbox.center.position.y = cluster.center_y_m;
    detection.bbox.center.position.z = 0.0;
    detection.bbox.center.orientation.w = 1.0;
    detection.bbox.size.x = cluster.size_x_m;
    detection.bbox.size.y = cluster.size_y_m;
    detection.bbox.size.z = 0.0;
  }
  clusters_publisher_->publish(output);
}

}  // namespace perception_lidar_clustering
