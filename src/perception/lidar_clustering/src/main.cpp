#include <memory>

#include "perception_lidar_clustering/lidar_clustering_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_lidar_clustering::LidarClusteringNode>());
  rclcpp::shutdown();
  return 0;
}
