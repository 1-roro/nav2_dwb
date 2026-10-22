cmake_minimum_required(VERSION 3.8)
project(simple_line_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(simple_line_mission_msgs REQUIRED)

set(dependencies
  rclcpp
  behaviortree_cpp_v3
  nav2_behavior_tree
  geometry_msgs
  lifecycle_msgs
  tf2
  tf2_ros
  ament_index_cpp
  std_msgs
  simple_line_mission_msgs
)

# 이 패키지의 BT 노드들
add_library(${PROJECT_NAME}_nodes SHARED src/mission_nodes.cpp)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_nodes ${dependencies})

# mission.xml 을 실행하는 노드
add_executable(mission_bt_runner src/mission_bt_runner.cpp)
target_link_libraries(mission_bt_runner ${PROJECT_NAME}_nodes)
ament_target_dependencies(mission_bt_runner ${dependencies})

install(TARGETS ${PROJECT_NAME}_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS mission_bt_runner DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/mission_node.py scripts/task_sender.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mission test/test_mission.cpp TIMEOUT 120)
  target_link_libraries(test_mission ${PROJECT_NAME}_nodes)
  ament_target_dependencies(test_mission ${dependencies})
  target_compile_definitions(test_mission PRIVATE
    MISSION_XML="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/mission.xml")

  ament_add_gtest(test_task_loop test/test_task_loop.cpp TIMEOUT 120)
  target_link_libraries(test_task_loop ${PROJECT_NAME}_nodes)
  ament_target_dependencies(test_task_loop ${dependencies})
  target_compile_definitions(test_task_loop PRIVATE
    TASK_LOOP_XML="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/task_loop.xml")
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_nodes)
ament_export_dependencies(${dependencies})
ament_package()
