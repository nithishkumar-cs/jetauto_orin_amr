cmake_minimum_required(VERSION 3.22)
project(perception_tracking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)

add_library(perception_tracking_core src/multi_object_tracker.cpp)
target_include_directories(perception_tracking_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(perception_tracking_core PUBLIC Eigen3::Eigen)

add_library(perception_tracking_node_lib src/tracking_node.cpp)
target_include_directories(perception_tracking_node_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(perception_tracking_node_lib perception_tracking_core)
ament_target_dependencies(perception_tracking_node_lib
  amr_interfaces geometry_msgs rclcpp tf2 tf2_geometry_msgs tf2_ros vision_msgs)

add_executable(tracking_node src/main.cpp)
target_link_libraries(tracking_node perception_tracking_node_lib)

install(TARGETS tracking_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_multi_object_tracker_unit test/test_multi_object_tracker_unit.cpp)
  target_link_libraries(test_multi_object_tracker_unit perception_tracking_core)
  set_tests_properties(test_multi_object_tracker_unit PROPERTIES LABELS "unit;gtest")

  ament_add_gtest(test_tracking_node_component test/test_tracking_node_component.cpp)
  target_link_libraries(test_tracking_node_component perception_tracking_node_lib)
  ament_target_dependencies(test_tracking_node_component
    amr_interfaces geometry_msgs rclcpp tf2 tf2_ros vision_msgs)
  set_tests_properties(test_tracking_node_component PROPERTIES LABELS "component;gtest")
endif()

ament_package()
