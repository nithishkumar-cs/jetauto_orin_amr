cmake_minimum_required(VERSION 3.22)
project(stop_zone_recovery)

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
find_package(tf2_ros REQUIRED)

add_library(stop_zone_recovery_node_lib src/stop_zone_recovery_node.cpp)
target_include_directories(stop_zone_recovery_node_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(stop_zone_recovery_node_lib geometry_msgs rclcpp sensor_msgs tf2_ros)

add_executable(stop_zone_recovery_node src/main.cpp)
target_link_libraries(stop_zone_recovery_node stop_zone_recovery_node_lib)
install(TARGETS stop_zone_recovery_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stop_zone_recovery_component test/test_stop_zone_recovery_component.cpp)
  target_link_libraries(test_stop_zone_recovery_component stop_zone_recovery_node_lib)
  ament_target_dependencies(test_stop_zone_recovery_component geometry_msgs rclcpp sensor_msgs tf2_ros)
  set_tests_properties(test_stop_zone_recovery_component PROPERTIES LABELS "component;gtest")
endif()

ament_package()
