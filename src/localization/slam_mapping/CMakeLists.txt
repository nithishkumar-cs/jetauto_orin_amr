cmake_minimum_required(VERSION 3.22)
project(localization_slam_mapping)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../../configs/localization/slam_mapping.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
