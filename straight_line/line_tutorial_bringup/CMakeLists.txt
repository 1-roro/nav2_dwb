cmake_minimum_required(VERSION 3.8)
project(line_tutorial_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch params behavior_trees worlds maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/cte_monitor.py scripts/send_scenario.py DESTINATION lib/${PROJECT_NAME})

ament_package()
