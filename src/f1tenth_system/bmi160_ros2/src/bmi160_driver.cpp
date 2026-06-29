#include "bmi160_ros2/bmi160_driver.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cerrno>
#include <cstring>
#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>
#include <cmath>

namespace bmi160_ros2
{

BMI160Driver::BMI160Driver(const std::string & i2c_device, uint8_t i2c_addr)
: i2c_device_(i2c_device),
  i2c_addr_(i2c_addr),
  i2c_fd_(-1),
  is_connected_(false),
  gyro_offset_x_(0.0),
  gyro_offset_y_(0.0),
  gyro_offset_z_(0.0),
  accel_scale_(0.0),
  gyro_scale_(0.0)
{
}

BMI160Driver::~BMI160Driver()
{
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
  }
}

bool BMI160Driver::initialize()
{
  // Open I2C device
  i2c_fd_ = open(i2c_device_.c_str(), O_RDWR);
  if (i2c_fd_ < 0) {
    const int error = errno;
    std::cerr << "Failed to open I2C device: " << i2c_device_
              << " (" << std::strerror(error) << ")" << std::endl;
    return false;
  }

  const uint8_t requested_addr = i2c_addr_;
  const uint8_t alternate_addr =
    requested_addr == BMI160_ADDR_DEFAULT ? BMI160_ADDR_ALT : BMI160_ADDR_DEFAULT;

  if (!probe_i2c_address(requested_addr) && !probe_i2c_address(alternate_addr)) {
    std::cerr << "Failed to find BMI160 on " << i2c_device_
              << " at addresses 0x" << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<int>(requested_addr) << " or 0x"
              << static_cast<int>(alternate_addr) << std::dec << std::setfill(' ')
              << ". Check I2C bus, wiring, power, and permissions."
              << std::endl;
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }

  // Wake up accelerometer (Normal mode)
  if (!write_register(BMI160_CMD_REG, BMI160_CMD_ACC_NORMAL)) {
    std::cerr << "Failed to wake up accelerometer" << std::endl;
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Bosch recommends > 3.8ms

  // Wake up gyroscope (Normal mode)
  if (!write_register(BMI160_CMD_REG, BMI160_CMD_GYR_NORMAL)) {
    std::cerr << "Failed to wake up gyroscope" << std::endl;
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Bosch recommends > 80ms

  // Initialize sensors ODR and Range
  if (!init_accel_gyro()) {
    std::cerr << "Failed to configure accelerometer and gyroscope parameters" << std::endl;
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }

  is_connected_ = true;
  return true;
}

bool BMI160Driver::reconnect()
{
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
    i2c_fd_ = -1;
  }
  is_connected_ = false;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return initialize();
}

bool BMI160Driver::init_accel_gyro()
{
  // Configure Accelerometer: ±2g range
  if (!set_accel_range(AccelRange::RANGE_2G)) {
    return false;
  }

  // Set Accelerometer config: ODR = 100Hz, Bandwidth = normal (0x28)
  if (!write_register(BMI160_ACC_CONF_REG, 0x28)) {
    return false;
  }

  // Configure Gyroscope: ±250 dps range
  if (!set_gyro_range(GyroRange::RANGE_250DPS)) {
    return false;
  }

  // Set Gyroscope config: ODR = 100Hz, Bandwidth = normal (0x28)
  if (!write_register(BMI160_GYR_CONF_REG, 0x28)) {
    return false;
  }

  return true;
}

bool BMI160Driver::read_sensor_data(SensorData & data)
{
  if (i2c_fd_ < 0) {
    return false;
  }

  // Read Gyro and Accel raw data block (12 bytes starting at 0x0C)
  // Gyro (6 bytes: 0x0C to 0x11) + Accel (6 bytes: 0x12 to 0x17)
  uint8_t buffer[12];
  if (!read_registers(BMI160_GYRO_DATA_ADDR, buffer, 12)) {
    return false;
  }

  // Convert raw values (little-endian)
  int16_t raw_gyro_x = to_int16_le(&buffer[0]);
  int16_t raw_gyro_y = to_int16_le(&buffer[2]);
  int16_t raw_gyro_z = to_int16_le(&buffer[4]);

  int16_t raw_accel_x = to_int16_le(&buffer[6]);
  int16_t raw_accel_y = to_int16_le(&buffer[8]);
  int16_t raw_accel_z = to_int16_le(&buffer[10]);

  // Convert to metric units and apply gyro calibration offsets
  data.gyro_x = (raw_gyro_x * gyro_scale_) - gyro_offset_x_;
  data.gyro_y = (raw_gyro_y * gyro_scale_) - gyro_offset_y_;
  data.gyro_z = (raw_gyro_z * gyro_scale_) - gyro_offset_z_;

  data.accel_x = raw_accel_x * accel_scale_;
  data.accel_y = raw_accel_y * accel_scale_;
  data.accel_z = raw_accel_z * accel_scale_;

  // Read temperature data (2 bytes starting at 0x20)
  uint8_t temp_buf[2];
  if (read_registers(BMI160_TEMP_DATA_ADDR, temp_buf, 2)) {
    int16_t raw_temp = to_int16_le(temp_buf);
    // Temp LSB = 1/512 C, Offset = 23 C.
    data.temperature = (raw_temp * 0.001953125) + 23.0;
  } else {
    data.temperature = 0.0;
  }

  // Get current timestamp
  auto now = std::chrono::system_clock::now();
  auto ns_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(
    now.time_since_epoch());
  data.timestamp_ns = ns_since_epoch.count();

  return true;
}

bool BMI160Driver::set_accel_range(AccelRange range)
{
  if (!write_register(BMI160_ACC_RANGE_REG, static_cast<uint8_t>(range))) {
    return false;
  }

  switch (range) {
    case AccelRange::RANGE_2G:
      accel_scale_ = 9.80665 / 16384.0;
      break;
    case AccelRange::RANGE_4G:
      accel_scale_ = 9.80665 / 8192.0;
      break;
    case AccelRange::RANGE_8G:
      accel_scale_ = 9.80665 / 4096.0;
      break;
    case AccelRange::RANGE_16G:
      accel_scale_ = 9.80665 / 2048.0;
      break;
  }
  return true;
}

bool BMI160Driver::set_gyro_range(GyroRange range)
{
  if (!write_register(BMI160_GYR_RANGE_REG, static_cast<uint8_t>(range))) {
    return false;
  }

  switch (range) {
    case GyroRange::RANGE_2000DPS:
      gyro_scale_ = (2000.0 / 32768.0) * (M_PI / 180.0);
      break;
    case GyroRange::RANGE_1000DPS:
      gyro_scale_ = (1000.0 / 32768.0) * (M_PI / 180.0);
      break;
    case GyroRange::RANGE_500DPS:
      gyro_scale_ = (500.0 / 32768.0) * (M_PI / 180.0);
      break;
    case GyroRange::RANGE_250DPS:
      gyro_scale_ = (250.0 / 32768.0) * (M_PI / 180.0);
      break;
    case GyroRange::RANGE_125DPS:
      gyro_scale_ = (125.0 / 32768.0) * (M_PI / 180.0);
      break;
  }
  return true;
}

bool BMI160Driver::calibrate_gyro(uint32_t num_samples)
{
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;

  // Temporarily reset offsets so we read raw zero-calibrated gyro values
  gyro_offset_x_ = 0.0;
  gyro_offset_y_ = 0.0;
  gyro_offset_z_ = 0.0;

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  for (uint32_t i = 0; i < num_samples; ++i) {
    SensorData data;
    if (!read_sensor_data(data)) {
      std::cerr << "Failed to read sensor data during calibration" << std::endl;
      return false;
    }
    sum_x += data.gyro_x;
    sum_y += data.gyro_y;
    sum_z += data.gyro_z;

    if ((i + 1) % 200 == 0) {
      std::cerr << "  Calibration progress: " << (i + 1) << "/" << num_samples << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  gyro_offset_x_ = sum_x / num_samples;
  gyro_offset_y_ = sum_y / num_samples;
  gyro_offset_z_ = sum_z / num_samples;

  std::cerr << "Finished Gyro calibration. Offsets: X=" << gyro_offset_x_
            << " Y=" << gyro_offset_y_ << " Z=" << gyro_offset_z_ << std::endl;

  return true;
}

bool BMI160Driver::set_i2c_address(uint8_t i2c_addr)
{
  if (ioctl(i2c_fd_, I2C_SLAVE, i2c_addr) < 0) {
    const int error = errno;
    std::cerr << "Failed to set I2C address 0x" << std::hex << static_cast<int>(i2c_addr)
              << std::dec << " (" << std::strerror(error) << ")" << std::endl;
    return false;
  }

  i2c_addr_ = i2c_addr;
  return true;
}

bool BMI160Driver::probe_i2c_address(uint8_t i2c_addr)
{
  if (!set_i2c_address(i2c_addr)) {
    return false;
  }

  uint8_t chip_id = 0;
  if (!read_register(BMI160_CHIP_ID_REG, chip_id)) {
    std::cerr << "No response from BMI160 at address 0x" << std::hex
              << static_cast<int>(i2c_addr) << std::dec << std::endl;
    return false;
  }

  if (chip_id != BMI160_CHIP_ID_VAL) {
    std::cerr << "Unexpected CHIP_ID value at address 0x" << std::hex
              << static_cast<int>(i2c_addr) << ": 0x" << static_cast<int>(chip_id)
              << " (expected 0x" << static_cast<int>(BMI160_CHIP_ID_VAL) << ")"
              << std::dec << std::endl;
    return false;
  }

  std::cerr << "Found BMI160 at I2C address 0x" << std::hex
            << static_cast<int>(i2c_addr) << std::dec << std::endl;
  return true;
}

bool BMI160Driver::write_register(uint8_t reg, uint8_t value)
{
  uint8_t data[2] = {reg, value};
  if (write(i2c_fd_, data, 2) != 2) {
    const int error = errno;
    std::cerr << "Failed to write register 0x" << std::hex << static_cast<int>(reg)
              << std::dec << " (" << std::strerror(error) << ")" << std::endl;
    return false;
  }
  return true;
}

bool BMI160Driver::read_register(uint8_t reg, uint8_t & value)
{
  if (write(i2c_fd_, &reg, 1) != 1) {
    const int error = errno;
    std::cerr << "Failed to write register address 0x" << std::hex << static_cast<int>(reg)
              << std::dec << " (" << std::strerror(error) << ")" << std::endl;
    return false;
  }

  if (read(i2c_fd_, &value, 1) != 1) {
    const int error = errno;
    std::cerr << "Failed to read register 0x" << std::hex << static_cast<int>(reg)
              << std::dec << " (" << std::strerror(error) << ")" << std::endl;
    return false;
  }
  return true;
}

bool BMI160Driver::read_registers(uint8_t reg, uint8_t * data, size_t len)
{
  if (write(i2c_fd_, &reg, 1) != 1) {
    const int error = errno;
    std::cerr << "Failed to write register address 0x" << std::hex << static_cast<int>(reg)
              << std::dec << " (" << std::strerror(error) << ")" << std::endl;
    return false;
  }

  if (read(i2c_fd_, data, len) != static_cast<ssize_t>(len)) {
    const int error = errno;
    std::cerr << "Failed to read registers from 0x" << std::hex << static_cast<int>(reg)
              << std::dec << " (" << std::strerror(error) << ")" << std::endl;
    return false;
  }
  return true;
}

int16_t BMI160Driver::to_int16_le(const uint8_t * data)
{
  return static_cast<int16_t>((data[1] << 8) | data[0]);
}

}  // namespace bmi160_ros2
