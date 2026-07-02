#include "imu_ros2/drivers/bmi160/bmi160_driver.hpp"
#include "imu_ros2/drivers/bmi160/registers.hpp"
#include <cmath>
#include <thread>
#include <iostream>

namespace imu_ros2 {
namespace drivers {
namespace bmi160 {

BMI160Driver::BMI160Driver(const std::string& i2c_device, uint8_t i2c_addr)
  : state_(core::DriverState::DISCONNECTED),
    consecutive_errors_(0)
{
  i2c_ = std::make_unique<core::I2cTransport>(i2c_device, i2c_addr);
  update_scales(2, 250);
}

bool BMI160Driver::initialize()
{
  state_ = core::DriverState::INITIALIZING;
  
  if (!i2c_->open_device()) {
    state_ = core::DriverState::ERROR;
    return false;
  }

  uint8_t chip_id = 0;
  if (!i2c_->read_byte(CHIP_ID_REG, chip_id) || chip_id != CHIP_ID_VAL) {
    uint8_t alt_addr = (i2c_->address() == I2C_ADDR_DEFAULT) ? I2C_ADDR_ALT : I2C_ADDR_DEFAULT;
    i2c_->set_address(alt_addr);
    if (!i2c_->read_byte(CHIP_ID_REG, chip_id) || chip_id != CHIP_ID_VAL) {
      state_ = core::DriverState::ERROR;
      return false;
    }
  }

  i2c_->write_byte(CMD_REG, CMD_SOFT_RESET);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  i2c_->write_byte(CMD_REG, CMD_ACC_NORMAL);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  i2c_->write_byte(CMD_REG, CMD_GYR_NORMAL);
  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  if (!init_accel_gyro()) {
    state_ = core::DriverState::ERROR;
    return false;
  }

  state_ = core::DriverState::RUNNING;
  consecutive_errors_ = 0;
  return true;
}

bool BMI160Driver::reconnect()
{
  i2c_->close_device();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return initialize();
}

bool BMI160Driver::init_accel_gyro()
{
  set_accel_range(2);
  set_gyro_range(250);
  
  // 100Hz default
  i2c_->write_byte(ACC_CONF_REG, 0x28);
  i2c_->write_byte(GYR_CONF_REG, 0x28);
  
  return true;
}

bool BMI160Driver::read_data(core::RawImuData& data)
{
  data.valid = false;
  data.has_mag = false;

  if (state_ != core::DriverState::RUNNING) {
    return false;
  }

  // Burst read 12 bytes (gyro + accel starting at 0x0C)
  uint8_t buf[12];
  if (!i2c_->read_block_with_timestamp(GYRO_DATA_ADDR, buf, 12, data.timestamp)) {
    consecutive_errors_++;
    return false;
  }

  uint8_t temp_buf[2];
  if (!i2c_->read_block_with_timestamp(TEMP_DATA_ADDR, temp_buf, 2, data.timestamp)) {
    consecutive_errors_++;
    return false;
  }

  consecutive_errors_ = 0;
  data.valid = true;

  // Gyro (little endian)
  data.gyro[0] = to_int16_le(&buf[0]) * gyro_scale_;
  data.gyro[1] = to_int16_le(&buf[2]) * gyro_scale_;
  data.gyro[2] = to_int16_le(&buf[4]) * gyro_scale_;

  // Accel (little endian)
  data.accel[0] = to_int16_le(&buf[6]) * accel_scale_;
  data.accel[1] = to_int16_le(&buf[8]) * accel_scale_;
  data.accel[2] = to_int16_le(&buf[10]) * accel_scale_;

  int16_t raw_temp = to_int16_le(temp_buf);
  data.temperature = (raw_temp * 0.001953125) + 23.0;

  return true;
}

bool BMI160Driver::set_accel_range(int range_g)
{
  uint8_t val = 0x03; // 2G
  if (range_g == 4) val = 0x05;
  else if (range_g == 8) val = 0x08;
  else if (range_g == 16) val = 0x0C;
  
  if (i2c_->write_byte(ACC_RANGE_REG, val)) {
    update_scales(range_g, -1);
    return true;
  }
  return false;
}

bool BMI160Driver::set_gyro_range(int range_dps)
{
  uint8_t val = 0x03; // 250DPS
  if (range_dps == 125) val = 0x04;
  else if (range_dps == 500) val = 0x02;
  else if (range_dps == 1000) val = 0x01;
  else if (range_dps == 2000) val = 0x00;
  
  if (i2c_->write_byte(GYR_RANGE_REG, val)) {
    update_scales(-1, range_dps);
    return true;
  }
  return false;
}

bool BMI160Driver::set_output_data_rate(int hz)
{
  // Map hz to BMI160 ODR codes (simplified)
  uint8_t acc_odr = 0x28; // 100Hz
  if (hz >= 1600) acc_odr = 0x2C;
  else if (hz >= 800) acc_odr = 0x2B;
  else if (hz >= 400) acc_odr = 0x2A;
  else if (hz >= 200) acc_odr = 0x29;
  
  uint8_t gyr_odr = acc_odr; // Same for gyro
  
  bool ok = true;
  ok &= i2c_->write_byte(ACC_CONF_REG, acc_odr);
  ok &= i2c_->write_byte(GYR_CONF_REG, gyr_odr);
  return ok;
}

void BMI160Driver::update_scales(int accel_g, int gyro_dps)
{
  static int cur_accel = 2;
  static int cur_gyro = 250;
  
  if (accel_g > 0) cur_accel = accel_g;
  if (gyro_dps > 0) cur_gyro = gyro_dps;

  double accel_lsb = 32768.0 / cur_accel;
  accel_scale_ = 9.80665 / accel_lsb;

  double gyro_lsb = 32768.0 / cur_gyro;
  gyro_scale_ = (1.0 / gyro_lsb) * (M_PI / 180.0);
}

int16_t BMI160Driver::to_int16_le(const uint8_t* buf) const
{
  return (int16_t)((buf[1] << 8) | buf[0]);
}

} // namespace bmi160
} // namespace drivers
} // namespace imu_ros2
