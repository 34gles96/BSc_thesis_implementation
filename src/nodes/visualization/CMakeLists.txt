
cmake_minimum_required(VERSION 3.8)
project(visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui)
find_package(yaml-cpp REQUIRED)

# Executable
add_executable(image_visualization_node
  src/visual_subscriber.cpp
)

# ROS 2 dependencies
ament_target_dependencies(image_visualization_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# OpenCV dependencies
target_include_directories(image_visualization_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(image_visualization_node
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

# Install executable
install(TARGETS image_visualization_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
