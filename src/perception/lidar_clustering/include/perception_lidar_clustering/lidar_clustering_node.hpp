#ifndef PERCEPTION_LIDAR_CLUSTERING__LIDAR_CLUSTERING_NODE_HPP_
#define PERCEPTION_LIDAR_CLUSTERING__LIDAR_CLUSTERING_NODE_HPP_

#include <memory>

#include "perception_lidar_clustering/laser_scan_clusterer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace perception_lidar_clustering
{

class LidarClusteringNode : public rclcpp::Node
{
public:
  explicit LidarClusteringNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr & message);

  LidarClusterer clusterer_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr clusters_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

}  // namespace perception_lidar_clustering

#endif  // PERCEPTION_LIDAR_CLUSTERING__LIDAR_CLUSTERING_NODE_HPP_
