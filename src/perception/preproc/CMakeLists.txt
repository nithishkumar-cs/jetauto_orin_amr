cmake_minimum_required(VERSION 3.22)
project(perception_preproc LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(cuda_common REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(perception_preproc_cuda SHARED
  src/image_preprocessor.cu
)

target_include_directories(perception_preproc_cuda
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(perception_preproc_cuda
  cuda_common
)

target_link_libraries(perception_preproc_cuda
  CUDA::cudart
)

set_target_properties(perception_preproc_cuda PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  POSITION_INDEPENDENT_CODE ON
)

add_executable(preproc_node
  src/preproc_node.cpp
)

target_include_directories(preproc_node
  PRIVATE include
)

ament_target_dependencies(preproc_node
  amr_interfaces
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(preproc_node
  perception_preproc_cuda
)

install(
  TARGETS perception_preproc_cuda preproc_node
  EXPORT export_perception_preproc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_perception_preproc HAS_LIBRARY_TARGET)
ament_export_dependencies(cuda_common)
ament_package()

