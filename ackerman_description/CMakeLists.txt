cmake_minimum_required(VERSION 3.5)
project(ackerman_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(

  DIRECTORY launch urdf meshes world config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()