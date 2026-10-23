cmake_minimum_required(VERSION 3.8)
project(bt_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bt_mission_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  nav2_behavior_tree
  geometry_msgs
  lifecycle_msgs
  tf2
  tf2_ros
  ament_index_cpp
  std_msgs
  sensor_msgs
  bt_mission_msgs
)

# 이 패키지의 BT 노드들
add_library(${PROJECT_NAME}_nodes SHARED src/mission_nodes.cpp)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_nodes ${dependencies})

# BT 준비 코드(bt_common)와 ExecuteTask 액션 서버(TaskServer)
add_library(${PROJECT_NAME}_bt SHARED src/bt_common.cpp src/task_server.cpp)
target_link_libraries(${PROJECT_NAME}_bt ${PROJECT_NAME}_nodes)
ament_target_dependencies(${PROJECT_NAME}_bt ${dependencies})

# BT XML 하나를 끝날 때까지 실행하는 노드 (task_loop.xml, 또는 bt_old_mission 의 mission.xml)
add_executable(mission_bt_runner src/mission_bt_runner.cpp)
target_link_libraries(mission_bt_runner ${PROJECT_NAME}_bt)
ament_target_dependencies(mission_bt_runner ${dependencies})

# 작업을 액션으로 받아 task.xml 로 실행하는 노드
add_executable(task_server src/task_server_main.cpp)
target_link_libraries(task_server ${PROJECT_NAME}_bt)
ament_target_dependencies(task_server ${dependencies})

# ---- BtActionServer 버전 (task_server_lifecycle) ----
# 이 패키지의 BT 노드를 플러그인 .so 로. BtActionServer 는 플러그인 이름으로만 노드를 등록한다
add_library(${PROJECT_NAME}_plugin SHARED src/bt_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME}_nodes)
target_compile_definitions(${PROJECT_NAME}_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(${PROJECT_NAME}_plugin ${dependencies})

add_library(${PROJECT_NAME}_lifecycle SHARED src/task_server_lifecycle.cpp)
target_link_libraries(${PROJECT_NAME}_lifecycle ${PROJECT_NAME}_bt)
ament_target_dependencies(${PROJECT_NAME}_lifecycle ${dependencies} nav2_util rclcpp_lifecycle)

add_executable(task_server_lifecycle src/task_server_lifecycle_main.cpp)
target_link_libraries(task_server_lifecycle ${PROJECT_NAME}_lifecycle)
ament_target_dependencies(task_server_lifecycle ${dependencies} nav2_util rclcpp_lifecycle)

install(
  TARGETS ${PROJECT_NAME}_nodes ${PROJECT_NAME}_bt ${PROJECT_NAME}_plugin ${PROJECT_NAME}_lifecycle
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS mission_bt_runner task_server task_server_lifecycle DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/task_sender.py scripts/task_client.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 가짜 Nav2 노드. 테스트가 링크하고, test_task_server_lifecycle 에서는 플러그인으로도 불러온다
  add_library(test_fake_nav2 SHARED test/test_fakes.cpp)
  target_compile_definitions(test_fake_nav2 PRIVATE BT_PLUGIN_EXPORT)
  ament_target_dependencies(test_fake_nav2 behaviortree_cpp_v3 geometry_msgs sensor_msgs)

  ament_add_gtest(test_stop_and_go test/test_stop_and_go.cpp TIMEOUT 120)
  target_link_libraries(test_stop_and_go ${PROJECT_NAME}_bt test_fake_nav2)
  ament_target_dependencies(test_stop_and_go ${dependencies})

  ament_add_gtest(test_task_loop test/test_task_loop.cpp TIMEOUT 120)
  target_link_libraries(test_task_loop ${PROJECT_NAME}_nodes test_fake_nav2)
  ament_target_dependencies(test_task_loop ${dependencies})
  target_compile_definitions(test_task_loop PRIVATE
    TASK_LOOP_XML="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/task_loop.xml")

  ament_add_gtest(test_task_server test/test_task_server.cpp TIMEOUT 120)
  target_link_libraries(test_task_server ${PROJECT_NAME}_bt test_fake_nav2)
  ament_target_dependencies(test_task_server ${dependencies})
  target_compile_definitions(test_task_server PRIVATE
    TASK_XML="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/task.xml")

  # 플러그인(bt_mission_plugin, test_fake_nav2)을 빌드 폴더에서 찾도록 라이브러리 경로를 더한다
  ament_add_gtest(test_task_server_lifecycle test/test_task_server_lifecycle.cpp TIMEOUT 120
    APPEND_LIBRARY_DIRS ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(test_task_server_lifecycle ${PROJECT_NAME}_lifecycle test_fake_nav2)
  ament_target_dependencies(test_task_server_lifecycle ${dependencies} nav2_util rclcpp_lifecycle)
  target_compile_definitions(test_task_server_lifecycle PRIVATE
    TASK_XML="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/task.xml")
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_nodes ${PROJECT_NAME}_bt ${PROJECT_NAME}_lifecycle)
ament_export_dependencies(${dependencies})
ament_package()
