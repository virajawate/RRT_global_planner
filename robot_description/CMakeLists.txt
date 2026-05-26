cmake_minimum_required(VERSION 3.22)
project(robot_description)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY 
  urdf
  launch
  DESTINATION share/${PROJECT_NAME}
)