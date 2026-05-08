cmake_minimum_required(VERSION 3.22)
project(logging_and_diagnostics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(health_aggregator_node
  src/health_aggregator_node.cpp
)

ament_target_dependencies(health_aggregator_node
  amr_interfaces
  rclcpp
)

install(TARGETS health_aggregator_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()

