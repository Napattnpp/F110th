#include "icm20948_ros2/icm20948_driver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

namespace icm20948_ros2
{

class ICM20948Node : public rclcpp::Node
{
public:
  explicit ICM20948Node(const rclcpp::NodeOptions & options)
  : rclcpp::Node("icm20948_node", options)
  {
    // Declare parameters
    std::string i2c_device = declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
    int i2c_addr = declare_parameter<int>("i2c_addr", 0x68);
    bool publish_tf = declare_parameter<bool>("publish_tf", false);
    bool calibrate_on_startup = declare_parameter<bool>("calibrate_on_startup", true);
    int calibration_samples = declare_parameter<int>("calibration_samples", 1000);

    // Create driver
    driver_ = std::make_unique<ICM20948Driver>(i2c_device, (uint8_t)i2c_addr);

    // Initialize driver
    RCLCPP_INFO(get_logger(), "Initializing ICM-20948 IMU...");
    if (!driver_->initialize()) {
      RCLCPP_FATAL(get_logger(), "Failed to initialize ICM-20948 driver");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "ICM-20948 initialized successfully");

    // Calibrate gyroscope if requested
    if (calibrate_on_startup) {
      RCLCPP_INFO(get_logger(), "Starting gyro calibration...");
      if (!driver_->calibrate_gyro(calibration_samples)) {
        RCLCPP_WARN(get_logger(), "Gyro calibration failed");
      } else {
        RCLCPP_INFO(get_logger(), "Gyro calibration completed");
      }
    }

    // Create publishers
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::QoS(10));
    yaw_pub_ = create_publisher<std_msgs::msg::Float64>("yaw_rad", rclcpp::QoS(10));

    // Create TF broadcaster if requested
    if (publish_tf) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

    // Initialize orientation filter state
    qw_ = 1.0;
    qx_ = 0.0;
    qy_ = 0.0;
    qz_ = 0.0;

    // Create timer for sensor reading
    timer_ = create_wall_timer(20ms, std::bind(&ICM20948Node::timer_callback, this));

    RCLCPP_INFO(get_logger(), "ICM-20948 node started");
  }

private:
  void timer_callback()
  {
    if (!driver_->is_connected()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "IMU disconnected, attempting reconnect...");
      if (!driver_->reconnect()) {
        return;
      }
      RCLCPP_INFO(get_logger(), "IMU reconnected successfully");
    }

    ICM20948Driver::SensorData data;
    if (!driver_->read_sensor_data(data)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to read sensor data");
      return;
    }

    // Update orientation using simple complementary filter
    update_orientation(data);

    // Publish IMU message
    publish_imu(data);

    // Publish yaw angle
    publish_yaw();

    // Publish TF if enabled
    if (tf_broadcaster_) {
      publish_tf();
    }
  }

  void update_orientation(const ICM20948Driver::SensorData & data)
  {
    // Simple complementary filter for orientation
    const float alpha = 0.98f;  // Complementary filter weight

    // Get roll and pitch from accelerometer
    float accel_roll = std::atan2(data.accel_y, data.accel_z);
    float accel_pitch = std::atan2(-data.accel_x, std::sqrt(data.accel_y * data.accel_y + data.accel_z * data.accel_z));

    // Get current roll and pitch from quaternion
    float quat_roll = std::atan2(
      2.0f * (qw_ * qx_ + qy_ * qz_),
      1.0f - 2.0f * (qx_ * qx_ + qy_ * qy_));
    float quat_pitch = std::asin(2.0f * (qw_ * qy_ - qz_ * qx_));
    float quat_yaw = std::atan2(
      2.0f * (qw_ * qz_ + qx_ * qy_),
      1.0f - 2.0f * (qy_ * qy_ + qz_ * qz_));

    // Complementary filter
    float filt_roll = alpha * (quat_roll + data.gyro_x * 0.02f) + (1.0f - alpha) * accel_roll;
    float filt_pitch = alpha * (quat_pitch + data.gyro_y * 0.02f) + (1.0f - alpha) * accel_pitch;
    float filt_yaw = quat_yaw + data.gyro_z * 0.02f;

    // Convert back to quaternion
    float cy = std::cos(filt_yaw * 0.5f);
    float sy = std::sin(filt_yaw * 0.5f);
    float cp = std::cos(filt_pitch * 0.5f);
    float sp = std::sin(filt_pitch * 0.5f);
    float cr = std::cos(filt_roll * 0.5f);
    float sr = std::sin(filt_roll * 0.5f);

    qw_ = cr * cp * cy + sr * sp * sy;
    qx_ = sr * cp * cy - cr * sp * sy;
    qy_ = cr * sp * cy + sr * cp * sy;
    qz_ = cr * cp * sy - sr * sp * cy;

    // Normalize quaternion
    float norm = std::sqrt(qw_ * qw_ + qx_ * qx_ + qy_ * qy_ + qz_ * qz_);
    if (norm > 0.0f) {
      qw_ /= norm;
      qx_ /= norm;
      qy_ /= norm;
      qz_ /= norm;
    }

    yaw_ = filt_yaw;
  }

  void publish_imu(const ICM20948Driver::SensorData & data)
  {
    auto imu_msg = sensor_msgs::msg::Imu();

    // Set header
    imu_msg.header.stamp = rclcpp::Time(data.timestamp_ns);
    imu_msg.header.frame_id = "imu_link";

    // Set orientation (quaternion)
    imu_msg.orientation.w = qw_;
    imu_msg.orientation.x = qx_;
    imu_msg.orientation.y = qy_;
    imu_msg.orientation.z = qz_;

    // Covariances (tuned for EKF fusion)
    imu_msg.orientation_covariance[0] = 0.02;
    imu_msg.orientation_covariance[4] = 0.02;
    imu_msg.orientation_covariance[8] = 0.02;

    // Set angular velocity
    imu_msg.angular_velocity.x = data.gyro_x;
    imu_msg.angular_velocity.y = data.gyro_y;
    imu_msg.angular_velocity.z = data.gyro_z;

    imu_msg.angular_velocity_covariance[0] = 0.01;
    imu_msg.angular_velocity_covariance[4] = 0.01;
    imu_msg.angular_velocity_covariance[8] = 0.01;

    // Set linear acceleration
    imu_msg.linear_acceleration.x = data.accel_x;
    imu_msg.linear_acceleration.y = data.accel_y;
    imu_msg.linear_acceleration.z = data.accel_z;

    imu_msg.linear_acceleration_covariance[0] = 0.1;
    imu_msg.linear_acceleration_covariance[4] = 0.1;
    imu_msg.linear_acceleration_covariance[8] = 0.1;

    imu_pub_->publish(imu_msg);
  }

  void publish_yaw()
  {
    auto yaw_msg = std_msgs::msg::Float64();
    yaw_msg.data = yaw_;
    yaw_pub_->publish(yaw_msg);
  }

  void publish_tf()
  {
    auto tf_msg = geometry_msgs::msg::TransformStamped();
    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = "base_link";
    tf_msg.child_frame_id = "imu_link";

    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;

    tf_msg.transform.rotation.w = qw_;
    tf_msg.transform.rotation.x = qx_;
    tf_msg.transform.rotation.y = qy_;
    tf_msg.transform.rotation.z = qz_;

    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::unique_ptr<ICM20948Driver> driver_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Orientation state
  float qw_, qx_, qy_, qz_;
  float yaw_;
};

}  // namespace icm20948_ros2

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(icm20948_ros2::ICM20948Node)
