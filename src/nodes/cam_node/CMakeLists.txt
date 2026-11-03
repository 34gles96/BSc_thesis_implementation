cmake_minimum_required(VERSION 3.8)
project(cam_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(message_filters REQUIRED)




# ==========================================================
# MATLAB CODER CAMERA TRANSFORMATION LIBRARY
# ==========================================================

set(CAM_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/include/cam_node/CamLib_v2/camera_transform"
)

# Collect generated MATLAB Coder C++ source files
file(GLOB CAM_SOURCES CONFIGURE_DEPENDS
    "${CAM_DIR}/*.cpp"
)

# Build generated sources into a static library
add_library(camera_transform STATIC
    ${CAM_SOURCES}
)

# Directory containing generated header files
target_include_directories(camera_transform PUBLIC
    "${CAM_DIR}"
)




# ==========================================
# MATLAB CODER LIBRARY PATHS
# ==========================================



# Include directories
include_directories(
  include
  include/cam_node
  ${YAML_CPP_INCLUDE_DIRS}
)

# Executable target
add_executable(subscriber_rx
  src/subscriber_rx.cpp
  src/desiredTrajectory.cpp
  src/fitLanePoly.cpp
  src/pinholeBackProjection.cpp
  src/readConfig.cpp
  src/initializeCameraMounting.cpp
  src/obstacleDetect.cpp
)

target_include_directories(subscriber_rx PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
   "${CMAKE_CURRENT_SOURCE_DIR}/include/cam_node"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

ament_target_dependencies(subscriber_rx
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
)

# Link OpenCV + YAML-CPP + MATLAB Coder Library
target_link_libraries(subscriber_rx
  ${OpenCV_LIBS}
  yaml-cpp
  ${YAML_CPP_LIBRARIES}
  camera_transform
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install executable
install(TARGETS
  subscriber_rx
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()