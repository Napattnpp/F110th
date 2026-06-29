cmake_minimum_required(VERSION 3.8)
project(bmi160_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include)

# Create library for driver
add_library(bmi160_driver SHARED
  src/bmi160_driver.cpp
)

target_include_directories(bmi160_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bmi160_driver
  rclcpp
)

# Create component node
add_library(bmi160_node SHARED
  src/bmi160_node.cpp
)

target_include_directories(bmi160_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(bmi160_node
  bmi160_driver
)

ament_target_dependencies(bmi160_node
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)

rclcpp_components_register_node(bmi160_node
  PLUGIN "bmi160_ros2::BMI160Node"
  EXECUTABLE bmi160_node_exe
)

# Install
install(TARGETS bmi160_driver bmi160_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS bmi160_node_exe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(bmi160_driver)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
