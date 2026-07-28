cmake_minimum_required(VERSION 3.22)
project(perception_detection_depth_projection)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

add_library(perception_detection_depth_projection_core src/depth_projector.cpp)
target_include_directories(perception_detection_depth_projection_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

add_library(detection_depth_projection_node_lib src/detection_depth_projection_node.cpp)
target_include_directories(detection_depth_projection_node_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(detection_depth_projection_node_lib perception_detection_depth_projection_core)
ament_target_dependencies(detection_depth_projection_node_lib
  message_filters rclcpp sensor_msgs vision_msgs)

add_executable(detection_depth_projection_node src/main.cpp)
target_link_libraries(detection_depth_projection_node detection_depth_projection_node_lib)

install(TARGETS detection_depth_projection_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_depth_projector test/test_depth_projector.cpp)
  target_link_libraries(test_depth_projector perception_detection_depth_projection_core)
endif()

ament_package()
