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
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(bt_mission_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml-cpp REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  nav2_behavior_tree
  geometry_msgs
  lifecycle_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
  std_msgs
  sensor_msgs
  bt_mission_msgs
)

# 이 패키지의 BT 노드들 (작업 노드, 도킹 공통 노드, ArUco 도킹 노드)
add_library(${PROJECT_NAME}_nodes SHARED src/mission_nodes.cpp src/dock_nodes.cpp src/aruco_dock_nodes.cpp)
target_include_directories(${PROJECT_NAME}_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_nodes ${dependencies})
target_link_libraries(${PROJECT_NAME}_nodes yaml-cpp)

# BT 준비 코드(bt_common)와 ExecuteTask 액션 서버(TaskServer)
add_library(${PROJECT_NAME}_bt SHARED src/bt_common.cpp src/task_server.cpp)
target_link_libraries(${PROJECT_NAME}_bt ${PROJECT_NAME}_nodes)
ament_target_dependencies(${PROJECT_NAME}_bt ${dependencies})

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
install(TARGETS task_server task_server_lifecycle DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/task_client.py scripts/aruco_detector.py scripts/aruco_teach.py
  tools/make_aruco_dock.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 가짜 Nav2 노드. 테스트가 링크하고, test_task_server_lifecycle 에서는 플러그인으로도 불러온다
  add_library(test_fake_nav2 SHARED test/test_fakes.cpp)
  target_compile_definitions(test_fake_nav2 PRIVATE BT_PLUGIN_EXPORT)
  ament_target_dependencies(test_fake_nav2 behaviortree_cpp_v3 geometry_msgs)

  # 도킹 계산 (ROS 없이)
  ament_add_gtest(test_dock_control test/test_dock_control.cpp)
  target_include_directories(test_dock_control PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_aruco_dock_control test/test_aruco_dock_control.cpp)
  target_include_directories(test_aruco_dock_control PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  # aruco_detector.py 의 계산 (합성 영상). launch_testing 의 pytest 플러그인은 이 pytest 와 맞지 않아 끈다
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_aruco_detector test/test_aruco_detector.py TIMEOUT 60
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)

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
