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

# ONNX Runtime — optional. Pre-built tarball typically extracted at /opt/onnxruntime.
# When found, USE_ONNXRUNTIME is defined and the YOLOv11 / DepthAnything paths build.
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS ${ONNXRUNTIME_ROOT}/include
        ${ONNXRUNTIME_ROOT}/include/onnxruntime
        ${ONNXRUNTIME_ROOT}/include/onnxruntime/core/session
        /opt/onnxruntime/include
        /opt/onnxruntime/include/onnxruntime
        /opt/onnxruntime/include/onnxruntime/core/session)
find_library(ONNXRUNTIME_LIB onnxruntime
  HINTS ${ONNXRUNTIME_ROOT}/lib /opt/onnxruntime/lib)

if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIB)
  set(HAVE_ONNXRUNTIME TRUE)
  message(STATUS "ONNX Runtime found: ${ONNXRUNTIME_LIB}")
else()
  set(HAVE_ONNXRUNTIME FALSE)
  message(WARNING "ONNX Runtime not found — YOLOv11 / DepthAnything paths will be disabled. "
                  "Install with scripts/install_onnxruntime.sh and rebuild.")
endif()

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
set(DETECTION_SRCS
  src/detection_node.cpp
  src/hsv_detector.cpp
)
if(HAVE_ONNXRUNTIME)
  list(APPEND DETECTION_SRCS src/yolo_v11.cpp)
endif()

add_executable(detection_node ${DETECTION_SRCS})
ament_target_dependencies(detection_node ${PERCEPTION_DEPS})
target_include_directories(detection_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(detection_node ${OpenCV_LIBS})
if(HAVE_ONNXRUNTIME)
  target_compile_definitions(detection_node PRIVATE USE_ONNXRUNTIME)
  target_include_directories(detection_node PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
  target_link_libraries(detection_node ${ONNXRUNTIME_LIB})
endif()
install(TARGETS detection_node DESTINATION lib/${PROJECT_NAME})

# --- depth_node (Phase 3) ---
set(DEPTH_SRCS src/depth_node.cpp)
if(HAVE_ONNXRUNTIME)
  list(APPEND DEPTH_SRCS src/depth_anything.cpp)
endif()
add_executable(depth_node ${DEPTH_SRCS})
ament_target_dependencies(depth_node ${PERCEPTION_DEPS})
target_include_directories(depth_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(depth_node ${OpenCV_LIBS})
if(HAVE_ONNXRUNTIME)
  target_compile_definitions(depth_node PRIVATE USE_ONNXRUNTIME)
  target_include_directories(depth_node PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
  target_link_libraries(depth_node ${ONNXRUNTIME_LIB})
endif()
install(TARGETS depth_node DESTINATION lib/${PROJECT_NAME})

# --- pose_estimation_node (Phase 4) ---
add_executable(pose_estimation_node src/pose_estimation_node.cpp)
ament_target_dependencies(pose_estimation_node ${PERCEPTION_DEPS})
target_include_directories(pose_estimation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(pose_estimation_node ${OpenCV_LIBS})
install(TARGETS pose_estimation_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models OPTIONAL)

if(BUILD_TESTING)
  # Run only the lightweight linters — cpplint/uncrustify enforce Google/ROS2
  # style which doesn't match this codebase's brace placement.
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_xmllint QUIET)
  find_package(ament_cmake_lint_cmake QUIET)
  if(ament_cmake_xmllint_FOUND)
    ament_xmllint()
  endif()
  if(ament_cmake_lint_cmake_FOUND)
    ament_lint_cmake()
  endif()

  ament_add_gtest(test_hsv_detector test/test_hsv_detector.cpp src/hsv_detector.cpp)
  target_include_directories(test_hsv_detector PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_hsv_detector ${OpenCV_LIBS})
  ament_target_dependencies(test_hsv_detector
    rclcpp std_msgs sensor_msgs geometry_msgs perception_bot_msgs)
endif()

ament_package()
