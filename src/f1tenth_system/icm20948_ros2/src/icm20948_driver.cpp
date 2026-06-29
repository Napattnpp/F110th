#include "icm20948_ros2/icm20948_driver.hpp"

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

namespace icm20948_ros2
{

ICM20948Driver::ICM20948Driver(const std::string & i2c_device, uint8_t i2c_addr)
: i2c_device_(i2c_device),
  i2c_addr_(i2c_addr),
  i2c_fd_(-1),
  is_connected_(false),
  gyro_offset_x_(0.0),
  gyro_offset_y_(0.0),
  gyro_offset_z_(0.0),
  accel_scale_(0.0),
  gyro_scale_(0.0),
  mag_scale_(0.0)
{
}

ICM20948Driver::~ICM20948Driver()
{
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
  }
}

bool ICM20948Driver::initialize()
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
    requested_addr == ICM20948_ADDR_DEFAULT ? ICM20948_ADDR_ALT : ICM20948_ADDR_DEFAULT;
  if (!probe_i2c_address(requested_addr) && !probe_i2c_address(alternate_addr)) {
    std::cerr << "Failed to find ICM-20948 on " << i2c_device_
              << " at addresses 0x" << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<int>(requested_addr) << " or 0x"
              << static_cast<int>(alternate_addr) << std::dec << std::setfill(' ')
              << ". Check I2C bus, address jumper, wiring, power, and permissions."
              << std::endl;
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }

  // Reset device
  if (!write_register(0, ICM20948_PWR_MGMT_1, 0x81)) {  // Reset + auto select best clock
    std::cerr << "Failed to reset device" << std::endl;
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Wake up device
  if (!write_register(0, ICM20948_PWR_MGMT_1, 0x01)) {  // Auto select best clock
    std::cerr << "Failed to wake up device" << std::endl;
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Initialize accelerometer and gyroscope
  if (!init_accel_gyro()) {
    std::cerr << "Failed to initialize accel/gyro" << std::endl;
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }

  // Initialize magnetometer
  if (!init_magnetometer()) {
    std::cerr << "Warning: Failed to initialize magnetometer" << std::endl;
    // Continue anyway, magnetometer is optional
  }

  is_connected_ = true;
  return true;
}

bool ICM20948Driver::reconnect()
{
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
    i2c_fd_ = -1;
  }
  is_connected_ = false;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return initialize();
}

bool ICM20948Driver::init_accel_gyro()
{
  // Select User Bank 2 for accel/gyro configuration
  if (!select_user_bank(2)) {
    return false;
  }

  // Configure accelerometer: ±2g, 1.125 kHz sample rate
  if (!write_register(2, ICM20948_ACCEL_CONFIG, 0x00)) {  // ±2g
    return false;
  }

  // Configure gyroscope: ±250 dps, 1.125 kHz sample rate
  if (!write_register(2, ICM20948_GYRO_CONFIG_1, 0x00)) {  // ±250 dps
    return false;
  }

  // Enable gyro and accel
  if (!write_register(0, ICM20948_PWR_MGMT_2, 0x00)) {  // All sensors on
    return false;
  }

  // Set scale factors
  update_scale_factors();

  return true;
}

bool ICM20948Driver::init_magnetometer()
{
  // Select User Bank 3 for I2C master control
  if (!select_user_bank(3)) {
    return false;
  }

  // Enable I2C master mode
  if (!write_register(3, ICM20948_I2C_MST_CTRL, 0x5D)) {  // 400kHz I2C, master mode enabled
    return false;
  }

  // Configure I2C slave 0 to read from AK09916
  if (!write_register(3, ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80)) {  // Read mode
    return false;
  }

  if (!write_register(3, ICM20948_I2C_SLV0_REG, AK09916_STATUS_1)) {
    return false;
  }

  if (!write_register(3, ICM20948_I2C_SLV0_CTRL, 0x89)) {  // Read 9 bytes, enable
    return false;
  }

  // Enable I2C slave delay
  if (!write_register(3, ICM20948_I2C_MST_DELAY_CTRL, 0x01)) {  // Delay on read
    return false;
  }

  mag_scale_ = 0.15f;  // AK09916 sensitivity: 0.15 microT per LSB

  return true;
}

bool ICM20948Driver::read_sensor_data(SensorData & data)
{
  if (!is_connected_) {
    return false;
  }

  // Select User Bank 0 for data reading
  if (!select_user_bank(0)) {
    is_connected_ = false;
    return false;
  }

  // Read accel data (6 bytes)
  uint8_t accel_data[6];
  if (!read_registers(0, ICM20948_ACCEL_XOUT_H, accel_data, 6)) {
    is_connected_ = false;
    return false;
  }

  // Read gyro data (6 bytes)
  uint8_t gyro_data[6];
  if (!read_registers(0, ICM20948_GYRO_XOUT_H, gyro_data, 6)) {
    is_connected_ = false;
    return false;
  }

  // Read temperature (2 bytes)
  uint8_t temp_data[2];
  if (!read_registers(0, ICM20948_TEMP_OUT_H, temp_data, 2)) {
    is_connected_ = false;
    return false;
  }

  // Read magnetometer data from external sensor data registers
  uint8_t mag_data[9];
  if (!read_registers(0, ICM20948_EXT_SLV_SENS_DATA_00, mag_data, 9)) {
    // Magnetometer read failed, but continue with accel/gyro
    data.mag_x = 0.0;
    data.mag_y = 0.0;
    data.mag_z = 0.0;
  } else {
    // Parse magnetometer data (little endian)
    int16_t mag_raw_x = (int16_t)((mag_data[2] << 8) | mag_data[1]);
    int16_t mag_raw_y = (int16_t)((mag_data[4] << 8) | mag_data[3]);
    int16_t mag_raw_z = (int16_t)((mag_data[6] << 8) | mag_data[5]);

    data.mag_x = mag_raw_x * mag_scale_;
    data.mag_y = mag_raw_y * mag_scale_;
    data.mag_z = mag_raw_z * mag_scale_;
  }

  // Convert accel data (big endian)
  int16_t accel_raw_x = to_int16(&accel_data[0]);
  int16_t accel_raw_y = to_int16(&accel_data[2]);
  int16_t accel_raw_z = to_int16(&accel_data[4]);

  data.accel_x = accel_raw_x * accel_scale_;
  data.accel_y = accel_raw_y * accel_scale_;
  data.accel_z = accel_raw_z * accel_scale_;

  // Convert gyro data (big endian)
  int16_t gyro_raw_x = to_int16(&gyro_data[0]);
  int16_t gyro_raw_y = to_int16(&gyro_data[2]);
  int16_t gyro_raw_z = to_int16(&gyro_data[4]);

  data.gyro_x = (gyro_raw_x * gyro_scale_) - gyro_offset_x_;
  data.gyro_y = (gyro_raw_y * gyro_scale_) - gyro_offset_y_;
  data.gyro_z = (gyro_raw_z * gyro_scale_) - gyro_offset_z_;

  // Convert temperature data
  int16_t temp_raw = to_int16(&temp_data[0]);
  data.temperature = (temp_raw / 333.87) + 21.0;  // From datasheet

  // Get current timestamp
  auto now = std::chrono::system_clock::now();
  auto ns_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(
    now.time_since_epoch());
  data.timestamp_ns = ns_since_epoch.count();

  return true;
}

bool ICM20948Driver::set_accel_range(AccelRange range)
{
  if (!select_user_bank(2)) {
    return false;
  }

  if (!write_register(2, ICM20948_ACCEL_CONFIG, (uint8_t)range)) {
    return false;
  }

  update_scale_factors();
  return true;
}

bool ICM20948Driver::set_gyro_range(GyroRange range)
{
  if (!select_user_bank(2)) {
    return false;
  }

  if (!write_register(2, ICM20948_GYRO_CONFIG_1, (uint8_t)range)) {
    return false;
  }

  update_scale_factors();
  return true;
}

bool ICM20948Driver::set_sample_rate(uint16_t hz)
{
  (void)hz;
  return true;
}

bool ICM20948Driver::calibrate_gyro(uint32_t num_samples)
{
  std::cerr << "Calibrating gyroscope with " << num_samples << " samples..." << std::endl;

  double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;

  for (uint32_t i = 0; i < num_samples; ++i) {
    SensorData data;
    if (!read_sensor_data(data)) {
      std::cerr << "Failed to read sensor data during calibration" << std::endl;
      return false;
    }
    sum_x += data.gyro_x;
    sum_y += data.gyro_y;
    sum_z += data.gyro_z;

    if ((i + 1) % 100 == 0) {
      std::cerr << "  Calibration: " << (i + 1) << "/" << num_samples << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  gyro_offset_x_ = sum_x / num_samples;
  gyro_offset_y_ = sum_y / num_samples;
  gyro_offset_z_ = sum_z / num_samples;

  std::cerr << "Gyro offsets: X=" << gyro_offset_x_ << " Y=" << gyro_offset_y_
            << " Z=" << gyro_offset_z_ << std::endl;

  return true;
}

bool ICM20948Driver::set_i2c_address(uint8_t i2c_addr)
{
  if (ioctl(i2c_fd_, I2C_SLAVE, i2c_addr) < 0) {
    const int error = errno;
    std::cerr << "Failed to set I2C address 0x" << std::hex << std::setw(2)
              << std::setfill('0') << static_cast<int>(i2c_addr) << std::dec
              << std::setfill(' ') << " (" << std::strerror(error) << ")"
              << std::endl;
    return false;
  }

  i2c_addr_ = i2c_addr;
  return true;
}

bool ICM20948Driver::probe_i2c_address(uint8_t i2c_addr)
{
  if (!set_i2c_address(i2c_addr)) {
    return false;
  }

  uint8_t who_am_i = 0;
  if (!read_register(0, ICM20948_WHO_AM_I, who_am_i)) {
    std::cerr << "No response from ICM-20948 at address 0x" << std::hex
              << std::setw(2) << std::setfill('0') << static_cast<int>(i2c_addr)
              << std::dec << std::setfill(' ') << std::endl;
    return false;
  }

  if (who_am_i != ICM20948_WHO_AM_I_VALUE) {
    std::cerr << "Unexpected WHO_AM_I value at address 0x" << std::hex
              << std::setw(2) << std::setfill('0') << static_cast<int>(i2c_addr)
              << ": 0x" << static_cast<int>(who_am_i) << " (expected 0x"
              << static_cast<int>(ICM20948_WHO_AM_I_VALUE) << ")" << std::dec
              << std::setfill(' ') << std::endl;
    return false;
  }

  std::cerr << "Found ICM-20948 at I2C address 0x" << std::hex << std::setw(2)
            << std::setfill('0') << static_cast<int>(i2c_addr) << std::dec
            << std::setfill(' ') << std::endl;
  return true;
}

bool ICM20948Driver::write_register(uint8_t bank, uint8_t reg, uint8_t value)
{
  if (!select_user_bank(bank)) {
    return false;
  }

  uint8_t data[2] = {reg, value};
  if (write(i2c_fd_, data, 2) != 2) {
    const int error = errno;
    std::cerr << "Failed to write register 0x" << std::hex << static_cast<int>(reg)
              << std::dec << " (" << std::strerror(error) << ")" << std::endl;
    return false;
  }

  return true;
}

bool ICM20948Driver::read_register(uint8_t bank, uint8_t reg, uint8_t & value)
{
  if (!select_user_bank(bank)) {
    return false;
  }

  if (write(i2c_fd_, &reg, 1) != 1) {
    const int error = errno;
    std::cerr << "Failed to write register address 0x" << std::hex
              << static_cast<int>(reg) << std::dec << " (" << std::strerror(error)
              << ")" << std::endl;
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

bool ICM20948Driver::read_registers(uint8_t bank, uint8_t reg, uint8_t * data, size_t len)
{
  if (!select_user_bank(bank)) {
    return false;
  }

  if (write(i2c_fd_, &reg, 1) != 1) {
    const int error = errno;
    std::cerr << "Failed to write register address 0x" << std::hex
              << static_cast<int>(reg) << std::dec << " (" << std::strerror(error)
              << ")" << std::endl;
    return false;
  }

  if (read(i2c_fd_, data, len) != (ssize_t)len) {
    const int error = errno;
    std::cerr << "Failed to read registers from 0x" << std::hex
              << static_cast<int>(reg) << std::dec << " (" << std::strerror(error)
              << ")" << std::endl;
    return false;
  }

  return true;
}

bool ICM20948Driver::select_user_bank(uint8_t bank)
{
  uint8_t bank_sel = (bank << 4) & 0xF0;
  uint8_t data[2] = {ICM20948_REG_BANK_SEL, bank_sel};

  if (write(i2c_fd_, data, 2) != 2) {
    const int error = errno;
    std::cerr << "Failed to select user bank " << static_cast<int>(bank)
              << " at I2C address 0x" << std::hex << std::setw(2)
              << std::setfill('0') << static_cast<int>(i2c_addr_) << std::dec
              << std::setfill(' ') << " (" << std::strerror(error) << ")"
              << std::endl;
    return false;
  }

  return true;
}

int16_t ICM20948Driver::to_int16(const uint8_t * data)
{
  return (int16_t)((data[0] << 8) | data[1]);
}

void ICM20948Driver::update_scale_factors()
{
  // Accel scale: ±2g = 16384 LSB/g
  accel_scale_ = 9.80665 / 16384.0;  // Convert to m/s^2

  // Gyro scale: ±250 dps = 131 LSB/dps
  gyro_scale_ = (250.0 / 32768.0) * (M_PI / 180.0);  // Convert to rad/s
}

}  // namespace icm20948_ros2
