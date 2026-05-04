cmake_minimum_required(VERSION 3.8)
project(perception_bot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)

if(BUILD_TESTING)
  find_package(ament_cmake_xmllint QUIET)
  find_package(ament_cmake_lint_cmake QUIET)
  if(ament_cmake_xmllint_FOUND)
    ament_xmllint()
  endif()
  if(ament_cmake_lint_cmake_FOUND)
    ament_lint_cmake()
  endif()
endif()

ament_package()
