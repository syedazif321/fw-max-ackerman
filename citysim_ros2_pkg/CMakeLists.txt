# Must be the very first line. 3.10 is safe for Humble.
cmake_minimum_required(VERSION 3.10) 

# Define the package name
project(citysim_ros2_pkg) 

# --- AMENT & ROS 2 SETUP ---

# Find required packages for building and dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) 
find_package(gazebo_ros REQUIRED) 
find_package(PythonLibs REQUIRED) 
find_package(ament_cmake_python REQUIRED)

# --- INSTALLATION OF ASSETS ---

# 1. Install Python launch files
# CORRECT: Use DIRECTORY to install the whole launch folder structure into share/
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

# 2. Install Models (for GAZEBO_MODEL_PATH)
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

# 3. Install Media/Materials (for GAZEBO_RESOURCE_PATH)
install(
  DIRECTORY media
  DESTINATION share/${PROJECT_NAME}
)

# 4. Install World files
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# --- PACKAGE EXPORT ---

# Final command to register the package with the ROS 2 environment
ament_package()