#include <memory>

#include "estop_gate/estop_gate_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<estop_gate::EstopGateNode>());
  rclcpp::shutdown();
  return 0;
}
