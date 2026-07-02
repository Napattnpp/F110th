cmake_minimum_required(VERSION 3.8)
project(imu_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

# Include directories (Standard ROS 2 package format)
include_directories(include)

# ------------------------------------------------------------------
# Core Framework Library (ROS-Independent)
# ------------------------------------------------------------------
add_library(imu_core
  src/core/i2c_transport.cpp
  src/core/driver_factory.cpp
  src/drivers/icm20948/icm20948_driver.cpp
  src/drivers/bmi160/bmi160_driver.cpp
  src/calibration/gyro_calibration.cpp
  src/calibration/accel_calibration.cpp
  src/calibration/mag_calibration.cpp
  src/diagnostics/diagnostics_tracker.cpp
)
target_include_directories(imu_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Intentionally NOT linking any ROS libraries here to enforce separation

# ------------------------------------------------------------------
# ROS Application Layer
# ------------------------------------------------------------------
add_library(imu_node_lib SHARED src/ros/imu_node.cpp)
target_include_directories(imu_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(imu_node_lib
  imu_core
)
ament_target_dependencies(imu_node_lib
  rclcpp
  rclcpp_components
  sensor_msgs
  diagnostic_updater
)
rclcpp_components_register_node(imu_node_lib
  PLUGIN "imu_ros2::ros_layer::ImuNode"
  EXECUTABLE imu_node
)

# ------------------------------------------------------------------
# Tools / Examples
# ------------------------------------------------------------------
add_executable(calibrate_gyro tools/calibrate_gyro.cpp)
target_link_libraries(calibrate_gyro imu_core)
# No ROS dependencies for the tool

# ------------------------------------------------------------------
# Install
# ------------------------------------------------------------------
install(TARGETS
  imu_core
  imu_node_lib
  calibrate_gyro
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(imu_core imu_node_lib)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs diagnostic_updater)

ament_package()
