cmake_minimum_required(VERSION 3.8)
project(perception_bot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(perception_bot_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# ONNX Runtime — expects ONNXRUNTIME_ROOT to be set, e.g.:
#   cmake -DONNXRUNTIME_ROOT=/opt/onnxruntime ...
# or set via CMAKE_PREFIX_PATH
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS ${ONNXRUNTIME_ROOT}/include/onnxruntime/core/session
        /opt/onnxruntime/include/onnxruntime/core/session)
find_library(ONNXRUNTIME_LIB onnxruntime
  HINTS ${ONNXRUNTIME_ROOT}/lib /opt/onnxruntime/lib)

set(PERCEPTION_DEPS
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
  message_filters
  tf2_ros
  tf2_geometry_msgs
  image_geometry
  perception_bot_msgs
)

# --- detection_node (Phase 2) ---
# add_executable(detection_node
#   src/detection_node.cpp
#   src/yolo_detector.cpp
#   src/onnx_utils.cpp
# )
# ament_target_dependencies(detection_node ${PERCEPTION_DEPS})
# target_include_directories(detection_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   ${ONNXRUNTIME_INCLUDE_DIR}
#   ${OpenCV_INCLUDE_DIRS}
# )
# target_link_libraries(detection_node ${ONNXRUNTIME_LIB} ${OpenCV_LIBS})
# install(TARGETS detection_node DESTINATION lib/${PROJECT_NAME})

# --- depth_node (Phase 3) ---
# add_executable(depth_node ...)

# --- pose_estimation_node (Phase 4) ---
# add_executable(pose_estimation_node ...)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
