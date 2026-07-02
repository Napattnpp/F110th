#ifndef IMU_ROS2__ROS__IMU_NODE_HPP_
#define IMU_ROS2__ROS__IMU_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include "imu_ros2/core/imu_driver.hpp"
#include "imu_ros2/calibration/gyro_calibration.hpp"
#include "imu_ros2/calibration/accel_calibration.hpp"
#include "imu_ros2/calibration/mag_calibration.hpp"
#include "imu_ros2/diagnostics/diagnostics_tracker.hpp"

#include <memory>

namespace imu_ros2 {
namespace ros_layer {

class ImuNode : public rclcpp::Node {
public:
  explicit ImuNode(const rclcpp::NodeOptions& options);

private:
  void init_parameters();
  void init_driver();
  void init_calibration();
  void timer_callback();
  
  void update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

  // Core library components
  std::unique_ptr<core::ImuDriver> driver_;
  calibration::GyroCalibration gyro_cal_;
  calibration::AccelCalibration accel_cal_;
  calibration::MagCalibration mag_cal_;
  diagnostics::DiagnosticsTracker diagnostics_tracker_;

  // ROS parameters
  std::string sensor_type_;
  std::string i2c_device_;
  int i2c_addr_;
  std::string frame_id_;
  int rate_hz_;
  
  // ROS infrastructure
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;

  // Pre-allocated messages for zero-allocation fast loop
  sensor_msgs::msg::Imu imu_msg_;
  sensor_msgs::msg::MagneticField mag_msg_;
};

} // namespace ros_layer
} // namespace imu_ros2

#endif // IMU_ROS2__ROS__IMU_NODE_HPP_
