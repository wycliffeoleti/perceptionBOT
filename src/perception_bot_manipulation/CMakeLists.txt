cmake_minimum_required(VERSION 3.8)
project(perception_bot_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(perception_bot_msgs REQUIRED)

set(MANIPULATION_DEPS
  rclcpp
  moveit_ros_planning_interface
  moveit_task_constructor_core
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  perception_bot_msgs
)

# --- manipulation_node (Phase 5) ---
# add_executable(manipulation_node
#   src/manipulation_node.cpp
#   src/sort_planner.cpp
#   src/pick_place_task.cpp
# )
# ament_target_dependencies(manipulation_node ${MANIPULATION_DEPS})
# target_include_directories(manipulation_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# )
# install(TARGETS manipulation_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
