cmake_minimum_required(VERSION 3.22)
project(cuda_common LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(CUDAToolkit REQUIRED)

add_library(cuda_common SHARED
  src/cuda_common.cpp
)

target_include_directories(cuda_common
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(cuda_common
  PUBLIC
    CUDA::cudart
)

set_target_properties(cuda_common PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

install(
  TARGETS cuda_common
  EXPORT export_cuda_common
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_cuda_common HAS_LIBRARY_TARGET)
ament_export_dependencies(CUDAToolkit)
ament_package()

