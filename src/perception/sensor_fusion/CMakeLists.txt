cmake_minimum_required(VERSION 3.22)
project(sensor_fusion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(obstacle_fusion_node
  src/obstacle_fusion_node.cpp
)

ament_target_dependencies(obstacle_fusion_node
  amr_interfaces
  rclcpp
)

install(TARGETS obstacle_fusion_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()

