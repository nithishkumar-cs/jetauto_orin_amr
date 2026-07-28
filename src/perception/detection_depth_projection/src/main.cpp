#include <memory>

#include "perception_detection_depth_projection/detection_depth_projection_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<perception_detection_depth_projection::DetectionDepthProjectionNode>());
  rclcpp::shutdown();
  return 0;
}
