cmake_minimum_required(VERSION 3.22)
project(robot_simulation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

ament_package()

install(DIRECTORY 
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)
