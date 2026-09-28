cmake_minimum_required(VERSION 3.22)
project(localization_odometry_fusion)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Deployment profiles live with the other robot-level settings in configs/.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../configs/localization/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
