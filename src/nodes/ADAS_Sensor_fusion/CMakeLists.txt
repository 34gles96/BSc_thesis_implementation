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



set(EKF_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/include/EKFlib_v4/EKF_func
)

add_library(vehicle_ekf STATIC
  ${EKF_DIR}/EKF_func.cpp
  ${EKF_DIR}/EKF_func_initialize.cpp
  ${EKF_DIR}/EKF_func_terminate.cpp
  ${EKF_DIR}/EKF_func_data.cpp
  ${EKF_DIR}/mrdivide_helper.cpp
  ${EKF_DIR}/rt_nonfinite.cpp
  ${EKF_DIR}/rtGetInf.cpp
  ${EKF_DIR}/rtGetNaN.cpp
)

target_include_directories(vehicle_ekf PUBLIC
  ${EKF_DIR}
)
# ==========================================
# EXECUTABLE
# ==========================================

add_executable(adas_subscriber
  src/ADASSubcriber.cpp
  src/myLog.cpp
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
  yaml-cpp
  vehicle_ekf
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
  find_package(yaml-cpp REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()