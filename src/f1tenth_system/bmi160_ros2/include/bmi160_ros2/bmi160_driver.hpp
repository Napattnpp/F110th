#ifndef BMI160_ROS2__BMI160_DRIVER_HPP_
#define BMI160_ROS2__BMI160_DRIVER_HPP_

#include <cstdint>
#include <string>
#include <array>
#include <vector>
#include <cmath>

namespace bmi160_ros2
{

// BMI160 I2C addresses
constexpr uint8_t BMI160_ADDR_DEFAULT = 0x68;  // SDO connected to GND
constexpr uint8_t BMI160_ADDR_ALT = 0x69;      // SDO connected to VDDIO (Default for DFRobot)

// Registers
constexpr uint8_t BMI160_CHIP_ID_REG = 0x00;
constexpr uint8_t BMI160_CHIP_ID_VAL = 0xD1;

constexpr uint8_t BMI160_ERR_REG = 0x02;
constexpr uint8_t BMI160_PMU_STATUS_REG = 0x03;

// Data Registers
constexpr uint8_t BMI160_GYRO_DATA_ADDR = 0x0C; // Gyro data starts at 0x0C (LSB) to 0x11 (MSB)
constexpr uint8_t BMI160_ACCEL_DATA_ADDR = 0x12; // Accel data starts at 0x12 (LSB) to 0x17 (MSB)
constexpr uint8_t BMI160_TEMP_DATA_ADDR = 0x20;  // Temp data starts at 0x20 (LSB) to 0x21 (MSB)

// Configuration Registers
constexpr uint8_t BMI160_ACC_CONF_REG = 0x40;
constexpr uint8_t BMI160_ACC_RANGE_REG = 0x41;
constexpr uint8_t BMI160_GYR_CONF_REG = 0x42;
constexpr uint8_t BMI160_GYR_RANGE_REG = 0x43;

// Command Register
constexpr uint8_t BMI160_CMD_REG = 0x7E;

// Commands
constexpr uint8_t BMI160_CMD_SOFT_RESET = 0xB6;
constexpr uint8_t BMI160_CMD_ACC_NORMAL = 0x11;
constexpr uint8_t BMI160_CMD_GYR_NORMAL = 0x15;

// Accel / Gyro ranges
enum class AccelRange : uint8_t
{
  RANGE_2G = 0x03,
  RANGE_4G = 0x05,
  RANGE_8G = 0x08,
  RANGE_16G = 0x0C
};

enum class GyroRange : uint8_t
{
  RANGE_2000DPS = 0x00,
  RANGE_1000DPS = 0x01,
  RANGE_500DPS = 0x02,
  RANGE_250DPS = 0x03,
  RANGE_125DPS = 0x04
};

class BMI160Driver
{
public:
  BMI160Driver(const std::string & i2c_device, uint8_t i2c_addr = BMI160_ADDR_ALT);
  ~BMI160Driver();

  // Initialization and connection
  bool initialize();
  bool is_connected() const { return is_connected_; }
  bool reconnect();

  // Data reading
  struct SensorData
  {
    double accel_x, accel_y, accel_z;  // m/s^2
    double gyro_x, gyro_y, gyro_z;     // rad/s
    double temperature;                // Celsius
    int64_t timestamp_ns;              // Nanoseconds
  };

  bool read_sensor_data(SensorData & data);

  // Configuration
  bool set_accel_range(AccelRange range);
  bool set_gyro_range(GyroRange range);
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

  // I2C communication
  bool set_i2c_address(uint8_t i2c_addr);
  bool probe_i2c_address(uint8_t i2c_addr);
  bool write_register(uint8_t reg, uint8_t value);
  bool read_register(uint8_t reg, uint8_t & value);
  bool read_registers(uint8_t reg, uint8_t * data, size_t len);

  // Initialization steps
  bool init_accel_gyro();

  // Helper methods
  int16_t to_int16_le(const uint8_t * data);
};

}  // namespace bmi160_ros2

#endif  // BMI160_ROS2__BMI160_DRIVER_HPP_
