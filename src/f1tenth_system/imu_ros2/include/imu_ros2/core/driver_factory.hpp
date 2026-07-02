#ifndef IMU_ROS2__CORE__DRIVER_FACTORY_HPP_
#define IMU_ROS2__CORE__DRIVER_FACTORY_HPP_

#include "imu_ros2/core/imu_driver.hpp"
#include <memory>
#include <string>

namespace imu_ros2 {
namespace core {

class DriverFactory {
public:
  /**
   * @brief Create an IMU driver based on the sensor type
   * @param sensor_type e.g., "icm20948", "bmi160"
   * @param i2c_device e.g., "/dev/i2c-7"
   * @param i2c_addr e.g., 0x68
   * @return unique_ptr to the driver, or nullptr if type is unknown
   */
  static std::unique_ptr<ImuDriver> create(const std::string& sensor_type, const std::string& i2c_device, uint8_t i2c_addr);
};

} // namespace core
} // namespace imu_ros2

#endif // IMU_ROS2__CORE__DRIVER_FACTORY_HPP_
