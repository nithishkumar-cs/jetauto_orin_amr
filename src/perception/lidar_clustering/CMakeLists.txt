cmake_minimum_required(VERSION 3.22)
project(perception_lidar_clustering)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

add_library(perception_lidar_clustering_core src/laser_scan_clusterer.cpp)
target_include_directories(perception_lidar_clustering_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

add_library(lidar_clustering_node_lib src/lidar_clustering_node.cpp)
target_include_directories(lidar_clustering_node_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(lidar_clustering_node_lib perception_lidar_clustering_core)
ament_target_dependencies(lidar_clustering_node_lib rclcpp sensor_msgs vision_msgs)

add_executable(lidar_clustering_node src/main.cpp)
target_link_libraries(lidar_clustering_node lidar_clustering_node_lib)

install(TARGETS lidar_clustering_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_laser_scan_clusterer_unit test/test_laser_scan_clusterer_unit.cpp)
  target_link_libraries(test_laser_scan_clusterer_unit perception_lidar_clustering_core)
  set_tests_properties(test_laser_scan_clusterer_unit PROPERTIES LABELS "unit;gtest")

  ament_add_gtest(test_lidar_clustering_node_component test/test_lidar_clustering_node_component.cpp)
  target_link_libraries(test_lidar_clustering_node_component lidar_clustering_node_lib)
  ament_target_dependencies(test_lidar_clustering_node_component rclcpp sensor_msgs vision_msgs)
  set_tests_properties(
    test_lidar_clustering_node_component PROPERTIES LABELS "component;gtest")
endif()

ament_package()
