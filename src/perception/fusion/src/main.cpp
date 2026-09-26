#include <memory>

#include "perception_fusion/fusion_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_fusion::FusionNode>());
  rclcpp::shutdown();
  return 0;
}
