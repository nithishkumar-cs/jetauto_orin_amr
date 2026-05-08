cmake_minimum_required(VERSION 3.22)
project(teleop_tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(twist_safety_relay_node
  src/twist_safety_relay_node.cpp
)

ament_target_dependencies(twist_safety_relay_node
  geometry_msgs
  rclcpp
)

install(TARGETS twist_safety_relay_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()

