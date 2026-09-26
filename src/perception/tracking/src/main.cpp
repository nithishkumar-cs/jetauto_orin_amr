#include <memory>

#include "perception_tracking/tracking_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_tracking::TrackingNode>());
  rclcpp::shutdown();
  return 0;
}
