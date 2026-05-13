cmake_minimum_required(VERSION 3.22)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

install(FILES
  robot_stack.launch.py
  system_modes.yaml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch_lib
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../configs/
  DESTINATION share/${PROJECT_NAME}/project_configs
)

ament_package()
