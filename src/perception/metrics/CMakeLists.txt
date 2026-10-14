cmake_minimum_required(VERSION 3.22)
project(perception_metrics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_metrics_reporter_component test/test_metrics_reporter_component.cpp)
  target_include_directories(test_metrics_reporter_component PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include")
  ament_target_dependencies(test_metrics_reporter_component
    builtin_interfaces diagnostic_msgs rclcpp)
  set_tests_properties(
    test_metrics_reporter_component PROPERTIES LABELS "component;gtest")
endif()

ament_export_include_directories(include)
ament_export_dependencies(builtin_interfaces diagnostic_msgs rclcpp)
ament_package()
