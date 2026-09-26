#include <memory>

#include "perception_detector/detector_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_detector::DetectorNode>());
  rclcpp::shutdown();
  return 0;
}
