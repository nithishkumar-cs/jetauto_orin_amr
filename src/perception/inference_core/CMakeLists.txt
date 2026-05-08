cmake_minimum_required(VERSION 3.22)
project(perception_inference)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(detector_runtime SHARED
  src/detector_backend.cpp
)

target_include_directories(detector_runtime
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    /usr/include/${CMAKE_LIBRARY_ARCHITECTURE}
)

ament_target_dependencies(detector_runtime
  amr_interfaces
  sensor_msgs
)

target_link_libraries(detector_runtime
  CUDA::cudart
  nvinfer
  nvinfer_plugin
)

add_executable(detector_node
  src/detector_node.cpp
)

target_include_directories(detector_node
  PRIVATE include
)

ament_target_dependencies(detector_node
  amr_interfaces
  rclcpp
  sensor_msgs
)

target_link_libraries(detector_node
  detector_runtime
)

install(
  TARGETS detector_runtime detector_node
  EXPORT export_detector_runtime
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_detector_runtime HAS_LIBRARY_TARGET)
ament_package()
