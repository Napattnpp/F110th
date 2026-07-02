#include "imu_ros2/core/driver_factory.hpp"
#include "imu_ros2/drivers/bmi160/bmi160_driver.hpp"
#include "imu_ros2/drivers/icm20948/icm20948_driver.hpp"

namespace imu_ros2 {
namespace core {

std::unique_ptr<ImuDriver> DriverFactory::create(const std::string& sensor_type, const std::string& i2c_device, uint8_t i2c_addr)
{
  if (sensor_type == "icm20948") {
    return std::make_unique<drivers::icm20948::ICM20948Driver>(i2c_device, i2c_addr);
  } else if (sensor_type == "bmi160") {
    return std::make_unique<drivers::bmi160::BMI160Driver>(i2c_device, i2c_addr);
  }
  
  return nullptr;
}

} // namespace core
} // namespace imu_ros2
