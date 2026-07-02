#ifndef IMU_ROS2__DRIVERS__BMI160__REGISTERS_HPP_
#define IMU_ROS2__DRIVERS__BMI160__REGISTERS_HPP_

#include <cstdint>

namespace imu_ros2 {
namespace drivers {
namespace bmi160 {

constexpr uint8_t I2C_ADDR_DEFAULT = 0x68;
constexpr uint8_t I2C_ADDR_ALT = 0x69;

constexpr uint8_t CHIP_ID_REG = 0x00;
constexpr uint8_t CHIP_ID_VAL = 0xD1;

// Data Registers
constexpr uint8_t GYRO_DATA_ADDR = 0x0C;
constexpr uint8_t ACCEL_DATA_ADDR = 0x12;
constexpr uint8_t TEMP_DATA_ADDR = 0x20;

// Config Registers
constexpr uint8_t ACC_CONF_REG = 0x40;
constexpr uint8_t ACC_RANGE_REG = 0x41;
constexpr uint8_t GYR_CONF_REG = 0x42;
constexpr uint8_t GYR_RANGE_REG = 0x43;

constexpr uint8_t CMD_REG = 0x7E;

constexpr uint8_t CMD_SOFT_RESET = 0xB6;
constexpr uint8_t CMD_ACC_NORMAL = 0x11;
constexpr uint8_t CMD_GYR_NORMAL = 0x15;

} // namespace bmi160
} // namespace drivers
} // namespace imu_ros2

#endif // IMU_ROS2__DRIVERS__BMI160__REGISTERS_HPP_
