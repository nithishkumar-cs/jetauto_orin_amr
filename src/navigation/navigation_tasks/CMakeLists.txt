cmake_minimum_required(VERSION 3.22)
project(navigation_tasks)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(waypoint_task_manager_node
  src/waypoint_task_manager_node.cpp
)

ament_target_dependencies(waypoint_task_manager_node
  geometry_msgs
  rclcpp
  std_msgs
)

install(TARGETS waypoint_task_manager_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()

