#include "bmi160_ros2/bmi160_driver.hpp"

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

namespace bmi160_ros2
{

  class BMI160Node : public rclcpp::Node
  {
  public:
    explicit BMI160Node(const rclcpp::NodeOptions &options)
        : rclcpp::Node("bmi160_node", options)
    {
      // Declare parameters
      // Note: DFRobot BMI160 is on bus 7 at address 0x69
      std::string i2c_device = declare_parameter<std::string>("i2c_device", "/dev/i2c-7");
      int i2c_addr = declare_parameter<int>("i2c_addr", 0x69);
      bool publish_tf = declare_parameter<bool>("publish_tf", false);
      bool calibrate_on_startup = declare_parameter<bool>("calibrate_on_startup", true);
      int calibration_samples = declare_parameter<int>("calibration_samples", 1000);
      // Dead-zone: gyro_z readings below this threshold (rad/s) are treated as 0.
      // Prevents residual bias from slowly integrating into yaw while the robot is stationary.
      gyro_z_deadzone_ = declare_parameter<double>("gyro_z_deadzone", 0.003);

      // Create driver
      driver_ = std::make_unique<BMI160Driver>(i2c_device, (uint8_t)i2c_addr);

      // Initialize driver
      RCLCPP_INFO(get_logger(), "Initializing BMI160 IMU on %s at 0x%02X...", i2c_device.c_str(), i2c_addr);
      if (!driver_->initialize())
      {
        RCLCPP_FATAL(get_logger(), "Failed to initialize BMI160 driver");
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO(get_logger(), "BMI160 initialized successfully");

      // Calibrate gyroscope if requested
      if (calibrate_on_startup)
      {
        RCLCPP_INFO(get_logger(), "Starting gyro calibration (keep robot static)...");
        if (!driver_->calibrate_gyro(calibration_samples))
        {
          RCLCPP_WARN(get_logger(), "Gyro calibration failed");
        }
        else
        {
          RCLCPP_INFO(get_logger(), "Gyro calibration completed");
        }
      }

      // Create publishers
      imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::QoS(10));
      yaw_pub_ = create_publisher<std_msgs::msg::Float64>("yaw_rad", rclcpp::QoS(10));

      // Create TF broadcaster if requested
      if (publish_tf)
      {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
      }

      // Initialize orientation filter state
      qw_ = 1.0;
      qx_ = 0.0;
      qy_ = 0.0;
      qz_ = 0.0;
      yaw_ = 0.0;

      // Create timer for sensor reading (20ms -> 50Hz)
      timer_ = create_wall_timer(20ms, std::bind(&BMI160Node::timer_callback, this));

      RCLCPP_INFO(get_logger(), "BMI160 node started");
    }

  private:
    void timer_callback()
    {
      if (!driver_->is_connected())
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "IMU disconnected, attempting reconnect...");
        if (!driver_->reconnect())
        {
          return;
        }
        RCLCPP_INFO(get_logger(), "IMU reconnected successfully");
      }

      BMI160Driver::SensorData data;
      if (!driver_->read_sensor_data(data))
      {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to read sensor data");
        return;
      }

      // Update orientation using complementary filter
      update_orientation(data);

      // Publish IMU message
      publish_imu(data);

      // Publish yaw angle
      publish_yaw();

      // Publish TF if enabled
      if (tf_broadcaster_)
      {
        publish_tf();
      }
    }

    void update_orientation(const BMI160Driver::SensorData &data)
    {
      // Simple complementary filter for orientation
      const float alpha = 0.98f; // Complementary filter weight (98% gyro, 2% accel)

      // Calculate dynamic dt using actual sensor timestamp
      float dt = 0.02f;
      if (last_timestamp_ns_ > 0)
      {
        dt = static_cast<float>(data.timestamp_ns - last_timestamp_ns_) / 1e9f;
      }
      last_timestamp_ns_ = data.timestamp_ns;

      // Get roll and pitch from accelerometer
      float accel_roll = std::atan2(data.accel_y, data.accel_z);
      float accel_pitch = std::atan2(-data.accel_x, std::sqrt(data.accel_y * data.accel_y + data.accel_z * data.accel_z));

      // Get current roll, pitch, yaw from quaternion
      float quat_roll = std::atan2(
          2.0f * (qw_ * qx_ + qy_ * qz_),
          1.0f - 2.0f * (qx_ * qx_ + qy_ * qy_));
      float quat_pitch = std::asin(2.0f * (qw_ * qy_ - qz_ * qx_));
      float quat_yaw = std::atan2(
          2.0f * (qw_ * qz_ + qx_ * qy_),
          1.0f - 2.0f * (qy_ * qy_ + qz_ * qz_));

      // Complementary filter
      float filt_roll = alpha * (quat_roll + data.gyro_x * dt) + (1.0f - alpha) * accel_roll;
      float filt_pitch = alpha * (quat_pitch + data.gyro_y * dt) + (1.0f - alpha) * accel_pitch;
      // Apply dead-zone to gyro_z: clamp to 0 if below threshold to prevent bias integration.
      float gyro_z_filtered = (std::abs(data.gyro_z) < static_cast<float>(gyro_z_deadzone_)) ? 0.0f : data.gyro_z;
      float filt_yaw = quat_yaw + gyro_z_filtered * dt;

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
      if (norm > 0.0f)
      {
        qw_ /= norm;
        qx_ /= norm;
        qy_ /= norm;
        qz_ /= norm;
      }

      yaw_ = filt_yaw;
    }

    void publish_imu(const BMI160Driver::SensorData &data)
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

      // Set angular velocity (gyro)
      imu_msg.angular_velocity.x = data.gyro_x;
      imu_msg.angular_velocity.y = data.gyro_y;
      // Apply dead-zone to published gyro_z so the EKF also receives a zeroed value at rest.
      imu_msg.angular_velocity.z = (std::abs(data.gyro_z) < static_cast<float>(gyro_z_deadzone_)) ? 0.0 : data.gyro_z;

      imu_msg.angular_velocity_covariance[0] = 0.01;
      imu_msg.angular_velocity_covariance[4] = 0.01;
      imu_msg.angular_velocity_covariance[8] = 0.01;

      // Set linear acceleration (accel)
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

    std::unique_ptr<BMI160Driver> driver_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Orientation state
    float qw_, qx_, qy_, qz_;
    float yaw_;
    uint64_t last_timestamp_ns_ = 0;
    // Dead-zone threshold for gyro_z (rad/s)
    double gyro_z_deadzone_ = 0.0001;
  };

} // namespace bmi160_ros2

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(bmi160_ros2::BMI160Node)
