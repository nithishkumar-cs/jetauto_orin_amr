#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "scan_health_gate/scan_health_gate_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_health_gate::ScanHealthGateNode>());
  rclcpp::shutdown();
  return 0;
}
