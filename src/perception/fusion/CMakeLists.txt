cmake_minimum_required(VERSION 3.22)
project(perception_fusion)

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
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(perception_fusion_core src/multi_sensor_fusion.cpp)
target_include_directories(perception_fusion_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(perception_fusion_core PUBLIC Eigen3::Eigen)

add_library(perception_fusion_node_lib src/fusion_node.cpp)
target_include_directories(perception_fusion_node_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(perception_fusion_node_lib perception_fusion_core)
ament_target_dependencies(perception_fusion_node_lib
  amr_interfaces geometry_msgs message_filters rclcpp tf2 tf2_geometry_msgs)

add_executable(fusion_node src/main.cpp)
target_link_libraries(fusion_node perception_fusion_node_lib)

install(TARGETS fusion_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_multi_sensor_fusion_unit test/test_multi_sensor_fusion_unit.cpp)
  target_link_libraries(test_multi_sensor_fusion_unit perception_fusion_core)
  set_tests_properties(test_multi_sensor_fusion_unit PROPERTIES LABELS "unit;gtest")

  ament_add_gtest(test_fusion_node_component test/test_fusion_node_component.cpp)
  target_link_libraries(test_fusion_node_component perception_fusion_node_lib)
  ament_target_dependencies(test_fusion_node_component
    amr_interfaces geometry_msgs message_filters rclcpp tf2 tf2_geometry_msgs)
  set_tests_properties(test_fusion_node_component PROPERTIES LABELS "component;gtest")
endif()

ament_package()
