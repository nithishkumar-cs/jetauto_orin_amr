cmake_minimum_required(VERSION 3.22)
project(estop_gate)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(estop_gate_node_lib src/estop_gate_node.cpp)
target_include_directories(estop_gate_node_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(estop_gate_node_lib geometry_msgs rclcpp std_msgs)

add_executable(estop_gate_node src/main.cpp)
target_link_libraries(estop_gate_node estop_gate_node_lib)
install(TARGETS estop_gate_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_estop_gate_node test/test_estop_gate_node.cpp)
  target_link_libraries(test_estop_gate_node estop_gate_node_lib)
  ament_target_dependencies(test_estop_gate_node geometry_msgs rclcpp std_msgs)
endif()
ament_package()
