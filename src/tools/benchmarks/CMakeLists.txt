cmake_minimum_required(VERSION 3.22)
project(benchmarks)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(latency_probe_node
  src/latency_probe_node.cpp
)

ament_target_dependencies(latency_probe_node
  amr_interfaces
  rclcpp
  sensor_msgs
)

install(TARGETS latency_probe_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config scripts DESTINATION share/${PROJECT_NAME})

ament_package()

