cmake_minimum_required(VERSION 3.22)
project(sensor_drivers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(amr_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(usb_camera_node
  src/usb_camera_node.cpp
)

target_include_directories(usb_camera_node
  PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(usb_camera_node
  amr_interfaces
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(usb_camera_node
  ${OpenCV_LIBS}
)

install(TARGETS usb_camera_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

