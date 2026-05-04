cmake_minimum_required(VERSION 3.8)
project(perception_bot_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(perception_bot_msgs REQUIRED)

set(MANIPULATION_DEPS
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_msgs
  shape_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  perception_bot_msgs
)

add_executable(manipulation_node
  src/manipulation_node.cpp
  src/sort_planner.cpp
)
ament_target_dependencies(manipulation_node ${MANIPULATION_DEPS})
target_include_directories(manipulation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS manipulation_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_xmllint QUIET)
  find_package(ament_cmake_lint_cmake QUIET)
  if(ament_cmake_xmllint_FOUND)
    ament_xmllint()
  endif()
  if(ament_cmake_lint_cmake_FOUND)
    ament_lint_cmake()
  endif()

  ament_add_gtest(test_class_to_bin test/test_class_to_bin.cpp src/sort_planner.cpp)
  target_include_directories(test_class_to_bin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_class_to_bin ${MANIPULATION_DEPS})
endif()

ament_package()
