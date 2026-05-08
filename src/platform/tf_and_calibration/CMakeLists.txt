cmake_minimum_required(VERSION 3.22)
project(tf_and_calibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(calibration_validator_node
  src/calibration_validator_node.cpp
)

ament_target_dependencies(calibration_validator_node
  amr_interfaces
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

install(TARGETS calibration_validator_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
