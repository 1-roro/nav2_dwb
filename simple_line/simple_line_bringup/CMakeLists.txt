cmake_minimum_required(VERSION 3.8)
project(simple_line_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch params worlds maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/send_waypoints.py scripts/follow_waypoints.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
