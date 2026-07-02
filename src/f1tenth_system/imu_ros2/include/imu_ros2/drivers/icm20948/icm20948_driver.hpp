#ifndef IMU_ROS2__DRIVERS__ICM20948__ICM20948_DRIVER_HPP_
#define IMU_ROS2__DRIVERS__ICM20948__ICM20948_DRIVER_HPP_

#include "imu_ros2/core/imu_driver.hpp"
#include "imu_ros2/core/i2c_transport.hpp"
#include <memory>
#include <string>

namespace imu_ros2 {
namespace drivers {
namespace icm20948 {

class ICM20948Driver : public core::ImuDriver {
public:
  ICM20948Driver(const std::string& i2c_device, uint8_t i2c_addr);
  ~ICM20948Driver() override = default;

  bool initialize() override;
  bool reconnect() override;
  bool read_data(core::RawImuData& data) override;

  bool set_accel_range(int range_g) override;
  bool set_gyro_range(int range_dps) override;
  bool set_output_data_rate(int hz) override;

  bool has_magnetometer() const override { return true; }
  std::string sensor_name() const override { return "ICM-20948"; }
  core::DriverState get_state() const override { return state_; }
  uint32_t get_consecutive_errors() const override { return consecutive_errors_; }

private:
  std::unique_ptr<core::I2cTransport> i2c_;
  core::DriverState state_;
  uint32_t consecutive_errors_;
  uint8_t current_bank_;

  double accel_scale_;
  double gyro_scale_;
  double mag_scale_;

  bool select_bank(uint8_t bank);
  bool write_reg(uint8_t bank, uint8_t reg, uint8_t val);
  bool read_reg(uint8_t bank, uint8_t reg, uint8_t& val);
  
  bool init_accel_gyro();
  bool init_magnetometer();
  void update_scales(int accel_g, int gyro_dps);

  int16_t to_int16(const uint8_t* buf) const;
};

} // namespace icm20948
} // namespace drivers
} // namespace imu_ros2

#endif // IMU_ROS2__DRIVERS__ICM20948__ICM20948_DRIVER_HPP_
