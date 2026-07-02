#ifndef IMU_ROS2__DRIVERS__BMI160__BMI160_DRIVER_HPP_
#define IMU_ROS2__DRIVERS__BMI160__BMI160_DRIVER_HPP_

#include "imu_ros2/core/imu_driver.hpp"
#include "imu_ros2/core/i2c_transport.hpp"
#include <memory>
#include <string>

namespace imu_ros2 {
namespace drivers {
namespace bmi160 {

class BMI160Driver : public core::ImuDriver {
public:
  BMI160Driver(const std::string& i2c_device, uint8_t i2c_addr);
  ~BMI160Driver() override = default;

  bool initialize() override;
  bool reconnect() override;
  bool read_data(core::RawImuData& data) override;

  bool set_accel_range(int range_g) override;
  bool set_gyro_range(int range_dps) override;
  bool set_output_data_rate(int hz) override;

  bool has_magnetometer() const override { return false; }
  std::string sensor_name() const override { return "BMI160"; }
  core::DriverState get_state() const override { return state_; }
  uint32_t get_consecutive_errors() const override { return consecutive_errors_; }

private:
  std::unique_ptr<core::I2cTransport> i2c_;
  core::DriverState state_;
  uint32_t consecutive_errors_;

  double accel_scale_;
  double gyro_scale_;

  bool init_accel_gyro();
  void update_scales(int accel_g, int gyro_dps);
  int16_t to_int16_le(const uint8_t* buf) const;
};

} // namespace bmi160
} // namespace drivers
} // namespace imu_ros2

#endif // IMU_ROS2__DRIVERS__BMI160__BMI160_DRIVER_HPP_
