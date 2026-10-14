#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "stop_zone_recovery/stop_zone_recovery_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stop_zone_recovery::StopZoneRecoveryNode>());
  rclcpp::shutdown();
  return 0;
}
