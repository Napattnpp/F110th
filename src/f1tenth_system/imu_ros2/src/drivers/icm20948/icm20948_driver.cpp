#include "imu_ros2/drivers/icm20948/icm20948_driver.hpp"
#include "imu_ros2/drivers/icm20948/registers.hpp"
#include <cmath>
#include <thread>
#include <iostream>

namespace imu_ros2 {
namespace drivers {
namespace icm20948 {

ICM20948Driver::ICM20948Driver(const std::string& i2c_device, uint8_t i2c_addr)
  : state_(core::DriverState::DISCONNECTED),
    consecutive_errors_(0),
    current_bank_(255)
{
  i2c_ = std::make_unique<core::I2cTransport>(i2c_device, i2c_addr);
  update_scales(2, 250); // defaults
}

bool ICM20948Driver::initialize()
{
  state_ = core::DriverState::INITIALIZING;
  
  if (!i2c_->open_device()) {
    state_ = core::DriverState::ERROR;
    return false;
  }

  // Probe
  current_bank_ = 255;
  uint8_t whoami = 0;
  if (!read_reg(0, WHO_AM_I, whoami) || whoami != WHO_AM_I_VAL) {
    // Try alternate address
    uint8_t alt_addr = (i2c_->address() == I2C_ADDR_DEFAULT) ? I2C_ADDR_ALT : I2C_ADDR_DEFAULT;
    i2c_->set_address(alt_addr);
    current_bank_ = 255;
    if (!read_reg(0, WHO_AM_I, whoami) || whoami != WHO_AM_I_VAL) {
      state_ = core::DriverState::ERROR;
      return false;
    }
  }

  // Reset
  write_reg(0, PWR_MGMT_1, 0x81);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Wake & Auto Clock
  write_reg(0, PWR_MGMT_1, 0x01);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  if (!init_accel_gyro()) {
    state_ = core::DriverState::ERROR;
    return false;
  }

  init_magnetometer(); // Optional, ignore failure for now

  state_ = core::DriverState::RUNNING;
  consecutive_errors_ = 0;
  return true;
}

bool ICM20948Driver::reconnect()
{
  i2c_->close_device();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return initialize();
}

bool ICM20948Driver::init_accel_gyro()
{
  // All sensors on
  if (!write_reg(0, PWR_MGMT_2, 0x00)) return false;
  
  // ±2g, ±250dps defaults
  if (!write_reg(2, ACCEL_CONFIG, 0x01)) return false; // ±2g, DLPF
  if (!write_reg(2, GYRO_CONFIG_1, 0x01)) return false; // ±250dps, DLPF
  
  return true;
}

bool ICM20948Driver::init_magnetometer()
{
  // I2C Master mode
  if (!write_reg(0, USER_CTRL, 0x20)) return false; // Enable I2C Master
  if (!write_reg(3, I2C_MST_CTRL, 0x07)) return false; // 400kHz

  // Reset AK09916
  write_reg(3, I2C_SLV0_ADDR, AK09916_I2C_ADDR);
  write_reg(3, I2C_SLV0_REG, AK09916_CNTL_3);
  write_reg(3, I2C_SLV0_DO, 0x01);
  write_reg(3, I2C_SLV0_CTRL, 0x81);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Configure continuous mode 100Hz
  write_reg(3, I2C_SLV0_ADDR, AK09916_I2C_ADDR);
  write_reg(3, I2C_SLV0_REG, AK09916_CNTL_2);
  write_reg(3, I2C_SLV0_DO, 0x08);
  write_reg(3, I2C_SLV0_CTRL, 0x81);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Setup auto-read of mag data (ST1 to ST2, 9 bytes)
  write_reg(3, I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80); // Read
  write_reg(3, I2C_SLV0_REG, AK09916_STATUS_1);
  write_reg(3, I2C_SLV0_CTRL, 0x89); // Read 9 bytes, enabled

  mag_scale_ = 0.15; // 0.15 uT per LSB

  return true;
}

bool ICM20948Driver::read_data(core::RawImuData& data)
{
  data.valid = false;
  if (state_ != core::DriverState::RUNNING) {
    return false;
  }

  if (!select_bank(0)) {
    consecutive_errors_++;
    return false;
  }

  // Burst read 23 bytes: Accel(6), Gyro(6), Temp(2), Mag(9)
  uint8_t buf[23];
  if (!i2c_->read_block_with_timestamp(ACCEL_XOUT_H, buf, 23, data.timestamp)) {
    consecutive_errors_++;
    return false;
  }

  consecutive_errors_ = 0;
  data.valid = true;

  // Accel (big endian)
  data.accel[0] = to_int16(&buf[0]) * accel_scale_;
  data.accel[1] = to_int16(&buf[2]) * accel_scale_;
  data.accel[2] = to_int16(&buf[4]) * accel_scale_;

  // Gyro (big endian)
  data.gyro[0] = to_int16(&buf[6]) * gyro_scale_;
  data.gyro[1] = to_int16(&buf[8]) * gyro_scale_;
  data.gyro[2] = to_int16(&buf[10]) * gyro_scale_;

  // Temp
  int16_t temp_raw = to_int16(&buf[12]);
  data.temperature = (temp_raw / 333.87) + 21.0;

  // Mag (little endian, starts at buf[14] for EXT_SLV_SENS_DATA_00)
  // ST1 = buf[14], HXL = buf[15], etc.
  uint8_t st1 = buf[14];
  if (st1 & 0x01) { // Data ready
    int16_t mag_x = (int16_t)((buf[16] << 8) | buf[15]);
    int16_t mag_y = (int16_t)((buf[18] << 8) | buf[17]);
    int16_t mag_z = (int16_t)((buf[20] << 8) | buf[19]);
    // uint8_t st2 = buf[22];

    data.mag[0] = mag_x * mag_scale_;
    data.mag[1] = mag_y * mag_scale_;
    data.mag[2] = mag_z * mag_scale_;
    data.has_mag = true;
  } else {
    data.has_mag = false;
  }

  return true;
}

bool ICM20948Driver::set_accel_range(int range_g)
{
  uint8_t val = 0x01; // DLPF enabled
  if (range_g == 4) val |= (0x01 << 1);
  else if (range_g == 8) val |= (0x02 << 1);
  else if (range_g == 16) val |= (0x03 << 1);
  
  if (write_reg(2, ACCEL_CONFIG, val)) {
    update_scales(range_g, -1);
    return true;
  }
  return false;
}

bool ICM20948Driver::set_gyro_range(int range_dps)
{
  uint8_t val = 0x01; // DLPF enabled
  if (range_dps == 500) val |= (0x01 << 1);
  else if (range_dps == 1000) val |= (0x02 << 1);
  else if (range_dps == 2000) val |= (0x03 << 1);
  
  if (write_reg(2, GYRO_CONFIG_1, val)) {
    update_scales(-1, range_dps);
    return true;
  }
  return false;
}

bool ICM20948Driver::set_output_data_rate(int hz)
{
  // Sample rate = 1.125 kHz / (1 + DIV)
  if (hz <= 0 || hz > 1125) return false;
  
  uint16_t div = (1125 / hz) - 1;
  
  bool ok = true;
  ok &= write_reg(2, GYRO_SMPLRT_DIV, (uint8_t)div);
  ok &= write_reg(2, ACCEL_SMPLRT_DIV_1, (uint8_t)(div >> 8));
  ok &= write_reg(2, ACCEL_SMPLRT_DIV_2, (uint8_t)(div & 0xFF));
  return ok;
}

bool ICM20948Driver::select_bank(uint8_t bank)
{
  if (current_bank_ == bank) return true;
  uint8_t bank_sel = (bank << 4) & 0xF0;
  if (i2c_->write_byte(REG_BANK_SEL, bank_sel)) {
    current_bank_ = bank;
    return true;
  }
  return false;
}

bool ICM20948Driver::write_reg(uint8_t bank, uint8_t reg, uint8_t val)
{
  if (!select_bank(bank)) return false;
  return i2c_->write_byte(reg, val);
}

bool ICM20948Driver::read_reg(uint8_t bank, uint8_t reg, uint8_t& val)
{
  if (!select_bank(bank)) return false;
  return i2c_->read_byte(reg, val);
}

int16_t ICM20948Driver::to_int16(const uint8_t* buf) const
{
  return (int16_t)((buf[0] << 8) | buf[1]);
}

void ICM20948Driver::update_scales(int accel_g, int gyro_dps)
{
  static int cur_accel = 2;
  static int cur_gyro = 250;
  
  if (accel_g > 0) cur_accel = accel_g;
  if (gyro_dps > 0) cur_gyro = gyro_dps;

  // Accel: ±2g = 16384 LSB/g
  double accel_lsb = 16384.0 / (cur_accel / 2.0);
  accel_scale_ = 9.80665 / accel_lsb;

  // Gyro: ±250dps = 131 LSB/dps
  double gyro_lsb = 32768.0 / cur_gyro;
  gyro_scale_ = (1.0 / gyro_lsb) * (M_PI / 180.0);
}

} // namespace icm20948
} // namespace drivers
} // namespace imu_ros2
