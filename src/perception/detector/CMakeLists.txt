cmake_minimum_required(VERSION 3.22)
project(perception_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

add_library(perception_detector_core
  src/detector_backend.cpp
  src/backends/yolo/cpu_reference.cpp
  src/backends/yolo/postprocessing.cpp)
target_include_directories(perception_detector_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

# TensorRT is present on the Jetson deployment target but is deliberately not a
# required laptop dependency. Without it, the ROS/core tests still build and the
# production backend fails explicitly if somebody tries to instantiate it.
find_package(CUDAToolkit QUIET)
if(CUDAToolkit_FOUND)
  include(CheckLanguage)
  check_language(CUDA)
endif()
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  PATH_SUFFIXES aarch64-linux-gnu x86_64-linux-gnu)
find_library(TENSORRT_LIBRARY nvinfer)
find_library(TENSORRT_PLUGIN_LIBRARY nvinfer_plugin)
set(PERCEPTION_DETECTOR_TENSORRT_ENABLED FALSE)
set(PERCEPTION_DETECTOR_TENSORRT_SOURCES src/backends/yolo/tensorrt_backend.cpp)
if(CUDAToolkit_FOUND AND CMAKE_CUDA_COMPILER AND TENSORRT_INCLUDE_DIR AND TENSORRT_LIBRARY AND TENSORRT_PLUGIN_LIBRARY)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" AND NOT CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 87)
  endif()
  enable_language(CUDA)
  list(APPEND PERCEPTION_DETECTOR_TENSORRT_SOURCES
    src/runtime/tensorrt/session.cpp
    src/backends/yolo/cpu_preprocessor.cpp
    src/backends/yolo/cuda_preprocessor.cu)
  set(PERCEPTION_DETECTOR_TENSORRT_ENABLED TRUE)
endif()

add_library(perception_detector_tensorrt ${PERCEPTION_DETECTOR_TENSORRT_SOURCES})
target_include_directories(perception_detector_tensorrt PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(perception_detector_tensorrt perception_detector_core)

if(PERCEPTION_DETECTOR_TENSORRT_ENABLED)
  set_target_properties(perception_detector_tensorrt PROPERTIES
    CUDA_STANDARD 17
    CUDA_STANDARD_REQUIRED ON)
  target_compile_definitions(perception_detector_tensorrt PRIVATE
    PERCEPTION_DETECTOR_HAS_TENSORRT=1)
  target_include_directories(perception_detector_tensorrt PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${TENSORRT_INCLUDE_DIR})
  target_link_libraries(perception_detector_tensorrt
    CUDA::cudart
    ${TENSORRT_LIBRARY}
    ${TENSORRT_PLUGIN_LIBRARY})
  message(STATUS
    "perception_detector: TensorRT backend with selectable CPU/CUDA preprocessing enabled")
else()
  message(WARNING
    "perception_detector: TensorRT/CUDA development libraries not found; "
    "building a fail-fast unavailable backend")
endif()

add_library(perception_detector_backend_factory src/detector_backend_factory.cpp)
target_include_directories(perception_detector_backend_factory PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_include_directories(perception_detector_backend_factory PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(perception_detector_backend_factory
  perception_detector_core
  perception_detector_tensorrt)
ament_target_dependencies(perception_detector_backend_factory rclcpp)

add_library(perception_detector_node_lib src/detector_node.cpp)
target_include_directories(perception_detector_node_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(perception_detector_node_lib
  perception_detector_core
  perception_detector_backend_factory)
ament_target_dependencies(perception_detector_node_lib rclcpp sensor_msgs vision_msgs)

add_executable(detector_node src/main.cpp)
target_link_libraries(detector_node perception_detector_node_lib)

install(TARGETS detector_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_detector_core_unit test/test_detector_core_unit.cpp)
  target_include_directories(test_detector_core_unit PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(test_detector_core_unit perception_detector_core)
  set_tests_properties(test_detector_core_unit PROPERTIES LABELS "unit;gtest")

  ament_add_gtest(test_detector_node_component test/test_detector_node_component.cpp)
  target_link_libraries(test_detector_node_component perception_detector_node_lib)
  ament_target_dependencies(test_detector_node_component rclcpp sensor_msgs vision_msgs)
  set_tests_properties(test_detector_node_component PROPERTIES LABELS "component;gtest")

  if(PERCEPTION_DETECTOR_TENSORRT_ENABLED)
    ament_add_gtest(test_yolo_preprocessing_backends test/test_yolo_preprocessing_backends.cpp)
    target_include_directories(test_yolo_preprocessing_backends PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(test_yolo_preprocessing_backends
      perception_detector_core
      perception_detector_tensorrt
      CUDA::cudart)
    set_tests_properties(test_yolo_preprocessing_backends PROPERTIES LABELS "cuda;gtest")

    ament_add_gtest(test_detector_node_integration test/test_detector_node_integration.cpp)
    target_link_libraries(test_detector_node_integration
      perception_detector_node_lib
      CUDA::cudart)
    ament_target_dependencies(test_detector_node_integration rclcpp sensor_msgs vision_msgs)
    set_tests_properties(test_detector_node_integration PROPERTIES
      LABELS "integration;cuda;gtest"
      TIMEOUT 60)
  endif()
endif()

ament_package()
