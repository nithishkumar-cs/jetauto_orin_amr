cmake_minimum_required(VERSION 3.22)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

# Launch files live in launch/ per ROS convention. `ros2 launch robot_bringup
# robot_stack.launch.py` searches the share directory recursively, so the
# subdirectory does not change how it is invoked.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Python helpers imported by the launch file, installed as a SIBLING of launch/.
# That is why robot_stack.launch.py walks up one level to put its own parent
# directory on sys.path.
install(DIRECTORY launch_lib
  DESTINATION share/${PROJECT_NAME}
)

# Top-level configs/ (deployment + mode config), which is where
# configs/runtime_modes/system_modes.yaml arrives from — it is deliberately not
# installed out of this package. Three levels up because the package sits at
# src/orchestration/robot_bringup/. Everything is read from the *installed*
# path, so --symlink-install is what makes edits live without a rebuild.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../configs/
  DESTINATION share/${PROJECT_NAME}/project_configs
)

ament_package()
