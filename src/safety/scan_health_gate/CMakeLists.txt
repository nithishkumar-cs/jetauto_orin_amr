cmake_minimum_required(VERSION 3.22)
project(scan_health_gate)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(scan_health_gate_node_lib src/scan_health_gate_node.cpp)
target_include_directories(scan_health_gate_node_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(scan_health_gate_node_lib geometry_msgs rclcpp sensor_msgs)

add_executable(scan_health_gate_node src/main.cpp)
target_link_libraries(scan_health_gate_node scan_health_gate_node_lib)
install(TARGETS scan_health_gate_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scan_health_gate_component test/test_scan_health_gate_component.cpp)
  target_link_libraries(test_scan_health_gate_component scan_health_gate_node_lib)
  ament_target_dependencies(test_scan_health_gate_component geometry_msgs rclcpp sensor_msgs)
  set_tests_properties(test_scan_health_gate_component PROPERTIES LABELS "component;gtest")
endif()

ament_package()
