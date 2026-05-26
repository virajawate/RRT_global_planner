cmake_minimum_required(VERSION 3.22)
project(robot_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)

ament_package()

install(DIRECTORY 
  launch
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}
)
