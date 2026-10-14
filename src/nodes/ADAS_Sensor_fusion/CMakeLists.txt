cmake_minimum_required(VERSION 3.8)
project(ADAS_Sensor_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==========================================
# DEPENDENCIES
# ==========================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)

# ==========================================
# EXECUTABLE
# ==========================================

add_executable(adas_subscriber
  src/ADASSubcriber.cpp
)

# ==========================================
# INCLUDE DIRECTORIES
# ==========================================

target_include_directories(adas_subscriber PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ==========================================
# ROS 2 DEPENDENCIES
# ==========================================

ament_target_dependencies(adas_subscriber
  rclcpp
  sensor_msgs
  std_msgs
  message_filters
)

# ==========================================
# LINK LIBRARIES
# ==========================================

target_link_libraries(adas_subscriber
  Eigen3::Eigen
)

# ==========================================
# INSTALL HEADERS
# ==========================================

install(
  DIRECTORY include/
  DESTINATION include
)

# ==========================================
# INSTALL EXECUTABLE
# ==========================================

install(
  TARGETS adas_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# ==========================================
# TESTING
# ==========================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()