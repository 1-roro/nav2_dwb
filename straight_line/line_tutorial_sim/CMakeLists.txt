cmake_minimum_required(VERSION 3.8)
project(line_tutorial_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# 두 패키지의 헤더 전용 코어(line_builder.hpp, follower_core.hpp)만 쓴다
find_package(straight_line_planner REQUIRED)
find_package(line_follower_controller REQUIRED)

set(core_includes
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${straight_line_planner_INCLUDE_DIRS}
  ${line_follower_controller_INCLUDE_DIRS}
)

add_executable(step_matrix src/step_matrix.cpp)
target_include_directories(step_matrix PRIVATE ${core_includes})

install(TARGETS step_matrix DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_closed_loop test/test_closed_loop.cpp)
  target_include_directories(test_closed_loop PRIVATE ${core_includes})
endif()

ament_package()
