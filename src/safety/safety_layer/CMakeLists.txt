cmake_minimum_required(VERSION 3.22)
project(safety_layer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(safety_policy SHARED
  src/safety_policy.cpp
)

target_include_directories(safety_policy
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(safety_policy
  amr_interfaces
)

add_executable(safety_monitor_node
  src/safety_monitor_node.cpp
)

target_include_directories(safety_monitor_node
  PRIVATE include
)

ament_target_dependencies(safety_monitor_node
  amr_interfaces
  geometry_msgs
  rclcpp
)

target_link_libraries(safety_monitor_node
  safety_policy
)

install(
  TARGETS safety_policy safety_monitor_node
  EXPORT export_safety_policy
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_safety_policy HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_safety_policy test/test_safety_policy.cpp)
  target_link_libraries(test_safety_policy safety_policy)
  ament_target_dependencies(test_safety_policy amr_interfaces)
endif()

ament_package()

