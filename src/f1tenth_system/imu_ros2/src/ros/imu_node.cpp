#include "imu_ros2/ros/imu_node.hpp"
#include "imu_ros2/core/driver_factory.hpp"

using namespace std::chrono_literals;

namespace imu_ros2 {
namespace ros_layer {

ImuNode::ImuNode(const rclcpp::NodeOptions& options)
  : Node("imu_node", options)
{
  init_parameters();
  
  // Pre-allocate messages
  imu_msg_.header.frame_id = frame_id_;
  imu_msg_.orientation_covariance[0] = -1.0; // No orientation provided
  
  mag_msg_.header.frame_id = frame_id_;

  // Load covariances from parameters
  std::vector<double> accel_cov = declare_parameter<std::vector<double>>("accel_covariance", {0.0005, 0.0005, 0.0005});
  std::vector<double> gyro_cov = declare_parameter<std::vector<double>>("gyro_covariance", {0.00001, 0.00001, 0.00001});
  
  if (accel_cov.size() == 3) {
    imu_msg_.linear_acceleration_covariance[0] = accel_cov[0];
    imu_msg_.linear_acceleration_covariance[4] = accel_cov[1];
    imu_msg_.linear_acceleration_covariance[8] = accel_cov[2];
  }
  if (gyro_cov.size() == 3) {
    imu_msg_.angular_velocity_covariance[0] = gyro_cov[0];
    imu_msg_.angular_velocity_covariance[4] = gyro_cov[1];
    imu_msg_.angular_velocity_covariance[8] = gyro_cov[2];
  }

  init_calibration();
  init_driver();

  // Create publishers
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::QoS(10));
  if (driver_ && driver_->has_magnetometer()) {
    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", rclcpp::QoS(10));
  }

  // Diagnostics
  diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
  diagnostic_updater_->setHardwareID(sensor_type_);
  diagnostic_updater_->add("IMU Status", this, &ImuNode::update_diagnostics);

  // Start timer
  auto period = std::chrono::duration<double>(1.0 / rate_hz_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ImuNode::timer_callback, this)
  );

  RCLCPP_INFO(get_logger(), "IMU node started. Sensor: %s", sensor_type_.c_str());
}

void ImuNode::init_parameters()
{
  sensor_type_ = declare_parameter<std::string>("sensor_type", "icm20948");
  i2c_device_ = declare_parameter<std::string>("i2c_device", "/dev/i2c-7");
  i2c_addr_ = declare_parameter<int>("i2c_addr", 0x68);
  frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
  rate_hz_ = declare_parameter<int>("rate_hz", 200);
}

void ImuNode::init_calibration()
{
  gyro_cal_.set_bias(
    declare_parameter<double>("gyro_bias_x", 0.0),
    declare_parameter<double>("gyro_bias_y", 0.0),
    declare_parameter<double>("gyro_bias_z", 0.0)
  );

  accel_cal_.set_bias(
    declare_parameter<double>("accel_bias_x", 0.0),
    declare_parameter<double>("accel_bias_y", 0.0),
    declare_parameter<double>("accel_bias_z", 0.0)
  );

  std::vector<double> hard_iron = declare_parameter<std::vector<double>>("mag_hard_iron", {0.0, 0.0, 0.0});
  std::vector<double> soft_iron = declare_parameter<std::vector<double>>("mag_soft_iron", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});

  if (hard_iron.size() == 3) {
    mag_cal_.set_hard_iron(hard_iron[0], hard_iron[1], hard_iron[2]);
  }
  if (soft_iron.size() == 9) {
    mag_cal_.set_soft_iron(soft_iron.data());
  }
}

void ImuNode::init_driver()
{
  driver_ = core::DriverFactory::create(sensor_type_, i2c_device_, i2c_addr_);
  if (!driver_) {
    RCLCPP_FATAL(get_logger(), "Unknown sensor_type: %s", sensor_type_.c_str());
    throw std::runtime_error("Unknown sensor type");
  }

  if (!driver_->initialize()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize driver on start.");
  } else {
    driver_->set_accel_range(declare_parameter<int>("accel_range_g", 2));
    driver_->set_gyro_range(declare_parameter<int>("gyro_range_dps", 250));
    driver_->set_output_data_rate(rate_hz_);
  }
}

void ImuNode::timer_callback()
{
  if (!driver_) return;

  if (driver_->get_state() != core::DriverState::RUNNING) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Driver not running, attempting reconnect...");
    if (driver_->reconnect()) {
      RCLCPP_INFO(get_logger(), "Reconnected to IMU");
      diagnostics_tracker_.reset();
    }
    return;
  }

  core::RawImuData raw_data;
  if (!driver_->read_data(raw_data) || !raw_data.valid) {
    diagnostics_tracker_.record_read_failure();
    if (driver_->get_consecutive_errors() > 3) {
      RCLCPP_ERROR(get_logger(), "Multiple read failures. Forcing reconnect.");
      driver_->reconnect();
    }
    return;
  }

  diagnostics_tracker_.record_read_success(raw_data.timestamp, raw_data.temperature);

  // Apply calibration
  gyro_cal_.apply(raw_data);
  accel_cal_.apply(raw_data);
  mag_cal_.apply(raw_data);

  // Map monotonic timestamp to ROS time (best effort mapping)
  // For standard nodes, node->now() is generally sufficient.
  rclcpp::Time ros_time = this->now();
  
  imu_msg_.header.stamp = ros_time;
  imu_msg_.angular_velocity.x = raw_data.gyro[0];
  imu_msg_.angular_velocity.y = raw_data.gyro[1];
  imu_msg_.angular_velocity.z = raw_data.gyro[2];
  
  imu_msg_.linear_acceleration.x = raw_data.accel[0];
  imu_msg_.linear_acceleration.y = raw_data.accel[1];
  imu_msg_.linear_acceleration.z = raw_data.accel[2];

  imu_pub_->publish(imu_msg_);

  if (raw_data.has_mag && mag_pub_) {
    mag_msg_.header.stamp = ros_time;
    mag_msg_.magnetic_field.x = raw_data.mag[0] / 1e6; // Convert uT to Tesla
    mag_msg_.magnetic_field.y = raw_data.mag[1] / 1e6;
    mag_msg_.magnetic_field.z = raw_data.mag[2] / 1e6;
    mag_pub_->publish(mag_msg_);
  }
}

void ImuNode::update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (driver_->get_state() == core::DriverState::RUNNING) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Running");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Disconnected/Error");
  }

  stat.add("Actual Rate (Hz)", diagnostics_tracker_.actual_rate_hz());
  stat.add("Total Reads", std::to_string(diagnostics_tracker_.total_reads()));
  stat.add("Total Errors", std::to_string(diagnostics_tracker_.total_errors()));
  stat.add("Consecutive Errors", std::to_string(driver_->get_consecutive_errors()));
  stat.add("Temperature (C)", diagnostics_tracker_.last_temperature());
  stat.add("Min Cycle (ms)", diagnostics_tracker_.min_cycle_time_ms());
  stat.add("Max Cycle (ms)", diagnostics_tracker_.max_cycle_time_ms());
  stat.add("Mean Cycle (ms)", diagnostics_tracker_.mean_cycle_time_ms());
}

} // namespace ros_layer
} // namespace imu_ros2

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(imu_ros2::ros_layer::ImuNode)
