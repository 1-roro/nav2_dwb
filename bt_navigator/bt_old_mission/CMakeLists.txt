cmake_minimum_required(VERSION 3.8)
project(bt_old_mission)

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

set(dependencies
  rclcpp
  behaviortree_cpp_v3
  nav2_behavior_tree
  geometry_msgs
)

# MakePose, MakePoses. bt_mission 의 mission_bt_runner 가 extra_plugin_libs 로 불러오는 플러그인이다
add_library(${PROJECT_NAME}_nodes SHARED src/pose_nodes.cpp)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(${PROJECT_NAME}_nodes PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(${PROJECT_NAME}_nodes ${dependencies})

install(TARGETS ${PROJECT_NAME}_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(PROGRAMS scripts/mission_node.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(bt_mission REQUIRED)
  find_package(tf2_ros REQUIRED)
  ament_add_gtest(test_old_mission test/test_old_mission.cpp TIMEOUT 120)
  target_link_libraries(test_old_mission ${PROJECT_NAME}_nodes)
  ament_target_dependencies(test_old_mission ${dependencies} bt_mission tf2_ros)
  target_compile_definitions(test_old_mission PRIVATE
    MISSION_XML="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/mission.xml")
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_nodes)
ament_export_dependencies(${dependencies})
ament_package()
