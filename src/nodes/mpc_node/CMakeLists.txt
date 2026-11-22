cmake_minimum_required(VERSION 3.8)
project(mpc_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(beamng_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)



set(MPC_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/include/MPClib/mpcController
)

file(GLOB MPC_SOURCES CONFIGURE_DEPENDS
    "${MPC_DIR}/*.cpp"
)

add_library(mpc STATIC
    ${MPC_SOURCES}
)


target_include_directories(mpc PUBLIC
  ${MPC_DIR}
)




add_executable(mpc_subscriber
    src/MPCcontroller.cpp
    src/mpcLogger.cpp
    src/trajectoryConnector.cpp
    src/buildRefHorizont.cpp
)


target_include_directories(mpc_subscriber PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(mpc_subscriber
    mpc
    yaml-cpp
)

ament_target_dependencies(mpc_subscriber
    rclcpp
    sensor_msgs
    nav_msgs
    beamng_msgs
)

install(TARGETS
    mpc_subscriber
    DESTINATION lib/${PROJECT_NAME}
)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
