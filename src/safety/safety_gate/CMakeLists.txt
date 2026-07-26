cmake_minimum_required(VERSION 3.22)
project(safety_gate)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# --- Pure policy core: no ROS dependencies, deliberately. -------------------
# The ROS node (added in a later increment) will link this library.
add_library(safety_policy src/policy.cpp)
target_include_directories(safety_policy PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_compile_features(safety_policy PUBLIC cxx_std_17)

install(
  TARGETS safety_policy
  EXPORT export_safety_gate
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_policy test/test_policy.cpp)
  target_link_libraries(test_policy safety_policy)
endif()

ament_export_include_directories(include)
ament_export_libraries(safety_policy)
ament_export_targets(export_safety_gate HAS_LIBRARY_TARGET)
ament_package()
