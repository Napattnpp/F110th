#ifndef ICM20948_ROS2__ICM20948_DRIVER_HPP_
#define ICM20948_ROS2__ICM20948_DRIVER_HPP_

#include <cstdint>
#include <string>
#include <array>
#include <vector>
#include <cmath>

namespace icm20948_ros2
{

// ICM-20948 I2C address
constexpr uint8_t ICM20948_ADDR_DEFAULT = 0x68;  // AD0 = 0
constexpr uint8_t ICM20948_ADDR_ALT = 0x69;      // AD0 = 1

// Register map
constexpr uint8_t ICM20948_WHO_AM_I = 0x00;
constexpr uint8_t ICM20948_WHO_AM_I_VALUE = 0xEA;

// Bank select
constexpr uint8_t ICM20948_REG_BANK_SEL = 0x7F;

// User Bank 0 registers
constexpr uint8_t ICM20948_PWR_MGMT_1 = 0x06;
constexpr uint8_t ICM20948_PWR_MGMT_2 = 0x07;
constexpr uint8_t ICM20948_INT_PIN_CFG = 0x0F;
constexpr uint8_t ICM20948_INT_ENABLE = 0x10;
constexpr uint8_t ICM20948_INT_ENABLE_1 = 0x11;
constexpr uint8_t ICM20948_ACCEL_XOUT_H = 0x2D;
constexpr uint8_t ICM20948_GYRO_XOUT_H = 0x33;
constexpr uint8_t ICM20948_TEMP_OUT_H = 0x39;

// User Bank 2 registers (accelerometer/gyroscope config)
constexpr uint8_t ICM20948_ACCEL_CONFIG = 0x14;
constexpr uint8_t ICM20948_GYRO_CONFIG_1 = 0x01;
constexpr uint8_t ICM20948_GYRO_CONFIG_2 = 0x02;

// User Bank 3 registers (magnetometer)
constexpr uint8_t ICM20948_I2C_MST_CTRL = 0x01;
constexpr uint8_t ICM20948_I2C_MST_DELAY_CTRL = 0x02;
constexpr uint8_t ICM20948_I2C_SLV0_ADDR = 0x03;
constexpr uint8_t ICM20948_I2C_SLV0_REG = 0x04;
constexpr uint8_t ICM20948_I2C_SLV0_CTRL = 0x05;
constexpr uint8_t ICM20948_I2C_SLV0_DO = 0x06;
constexpr uint8_t ICM20948_EXT_SLV_SENS_DATA_00 = 0x3B;

// Magnetometer (AK09916) registers
constexpr uint8_t AK09916_I2C_ADDR = 0x0C;
constexpr uint8_t AK09916_WHO_AM_I = 0x00;
constexpr uint8_t AK09916_WHO_AM_I_VALUE = 0x09;
constexpr uint8_t AK09916_STATUS_1 = 0x10;
constexpr uint8_t AK09916_HXL = 0x11;
constexpr uint8_t AK09916_CNTL_2 = 0x31;
constexpr uint8_t AK09916_CNTL_3 = 0x32;

// Accel/Gyro full scale ranges
enum class AccelRange : uint8_t
{
  RANGE_2G = 0x00,
  RANGE_4G = 0x01,
  RANGE_8G = 0x02,
  RANGE_16G = 0x03
};

enum class GyroRange : uint8_t
{
  RANGE_250DPS = 0x00,
  RANGE_500DPS = 0x01,
  RANGE_1000DPS = 0x02,
  RANGE_2000DPS = 0x03
};

class ICM20948Driver
{
public:
  ICM20948Driver(const std::string & i2c_device, uint8_t i2c_addr = ICM20948_ADDR_DEFAULT);
  ~ICM20948Driver();

  // Initialization and connection
  bool initialize();
  bool is_connected() const { return is_connected_; }
  bool reconnect();

  // Data reading
  struct SensorData
  {
    double accel_x, accel_y, accel_z;  // m/s^2
    double gyro_x, gyro_y, gyro_z;     // rad/s
    double mag_x, mag_y, mag_z;        // microTesla
    double temperature;                // Celsius
    int64_t timestamp_ns;              // Nanoseconds
  };

  bool read_sensor_data(SensorData & data);

  // Configuration
  bool set_accel_range(AccelRange range);
  bool set_gyro_range(GyroRange range);
  bool set_sample_rate(uint16_t hz);
  bool calibrate_gyro(uint32_t num_samples = 1000);

  // Getters
  double get_accel_scale() const { return accel_scale_; }
  double get_gyro_scale() const { return gyro_scale_; }

private:
  std::string i2c_device_;
  uint8_t i2c_addr_;
  int i2c_fd_;
  bool is_connected_;

  // Calibration offsets
  double gyro_offset_x_, gyro_offset_y_, gyro_offset_z_;

  // Scale factors for conversion to SI units
  double accel_scale_;
  double gyro_scale_;
  double mag_scale_;

  // I2C communication
  bool set_i2c_address(uint8_t i2c_addr);
  bool probe_i2c_address(uint8_t i2c_addr);
  bool write_register(uint8_t bank, uint8_t reg, uint8_t value);
  bool read_register(uint8_t bank, uint8_t reg, uint8_t & value);
  bool read_registers(uint8_t bank, uint8_t reg, uint8_t * data, size_t len);

  // Bank selection
  bool select_user_bank(uint8_t bank);

  // Initialization steps
  bool init_accel_gyro();
  bool init_magnetometer();
  bool setup_dmp();  // DMP setup (optional for quaternion output)

  // Helper methods
  int16_t to_int16(const uint8_t * data);
  void update_scale_factors();
};

}  // namespace icm20948_ros2

#endif  // ICM20948_ROS2__ICM20948_DRIVER_HPP_
