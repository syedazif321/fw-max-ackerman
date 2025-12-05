cmake_minimum_required(VERSION 3.8)
project(fw_max_cartographer)

# Find all necessary ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(cartographer_ros REQUIRED)
# find_package(launch_ros REQUIRED)

# --- INSTALLATION SECTION ---

# 1. Install the Cartographer configuration files (.lua)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 2. Install the Python launch files (.py)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
# --- PACKAGE EXPORT ---

# This creates the necessary files for ROS 2 to find the package and its resources
ament_package()