#ifndef IMU_ROS2__DRIVERS__ICM20948__REGISTERS_HPP_
#define IMU_ROS2__DRIVERS__ICM20948__REGISTERS_HPP_

#include <cstdint>

namespace imu_ros2 {
namespace drivers {
namespace icm20948 {

// I2C Addresses
constexpr uint8_t I2C_ADDR_DEFAULT = 0x68;
constexpr uint8_t I2C_ADDR_ALT = 0x69;

// Bank Selection
constexpr uint8_t REG_BANK_SEL = 0x7F;

// --- USER BANK 0 ---
constexpr uint8_t WHO_AM_I = 0x00;
constexpr uint8_t WHO_AM_I_VAL = 0xEA;

constexpr uint8_t USER_CTRL = 0x03;
constexpr uint8_t PWR_MGMT_1 = 0x06;
constexpr uint8_t PWR_MGMT_2 = 0x07;
constexpr uint8_t INT_PIN_CFG = 0x0F;

// Data out registers (Burst read starts here)
constexpr uint8_t ACCEL_XOUT_H = 0x2D;
constexpr uint8_t GYRO_XOUT_H = 0x33;
constexpr uint8_t TEMP_OUT_H = 0x39;
constexpr uint8_t EXT_SLV_SENS_DATA_00 = 0x3B;

// --- USER BANK 2 ---
constexpr uint8_t GYRO_SMPLRT_DIV = 0x00;
constexpr uint8_t GYRO_CONFIG_1 = 0x01;
constexpr uint8_t GYRO_CONFIG_2 = 0x02;

constexpr uint8_t ACCEL_SMPLRT_DIV_1 = 0x10;
constexpr uint8_t ACCEL_SMPLRT_DIV_2 = 0x11;
constexpr uint8_t ACCEL_CONFIG = 0x14;
constexpr uint8_t ACCEL_CONFIG_2 = 0x15;

// --- USER BANK 3 ---
constexpr uint8_t I2C_MST_CTRL = 0x01;
constexpr uint8_t I2C_MST_DELAY_CTRL = 0x02;
constexpr uint8_t I2C_SLV0_ADDR = 0x03;
constexpr uint8_t I2C_SLV0_REG = 0x04;
constexpr uint8_t I2C_SLV0_CTRL = 0x05;
constexpr uint8_t I2C_SLV0_DO = 0x06;

// --- AK09916 Magnetometer ---
constexpr uint8_t AK09916_I2C_ADDR = 0x0C;
constexpr uint8_t AK09916_WHO_AM_I = 0x00;
constexpr uint8_t AK09916_WHO_AM_I_VAL = 0x09;
constexpr uint8_t AK09916_STATUS_1 = 0x10;
constexpr uint8_t AK09916_HXL = 0x11;
constexpr uint8_t AK09916_STATUS_2 = 0x18;
constexpr uint8_t AK09916_CNTL_2 = 0x31;
constexpr uint8_t AK09916_CNTL_3 = 0x32;

} // namespace icm20948
} // namespace drivers
} // namespace imu_ros2

#endif // IMU_ROS2__DRIVERS__ICM20948__REGISTERS_HPP_
