#ifndef IMU_ROS2__CORE__I2C_TRANSPORT_HPP_
#define IMU_ROS2__CORE__I2C_TRANSPORT_HPP_

#include <cstdint>
#include <string>
#include <cstddef>
#include <chrono>

namespace imu_ros2 {
namespace core {

/**
 * @brief Simple RAII wrapper for Linux I2C
 */
class I2cTransport {
public:
  I2cTransport(const std::string& device, uint8_t address);
  ~I2cTransport();

  bool open_device();
  void close_device();
  bool is_open() const;
  bool set_address(uint8_t addr);

  // Read/Write operations
  bool write_byte(uint8_t reg, uint8_t val);
  bool read_byte(uint8_t reg, uint8_t& val);
  
  /**
   * @brief Read a block of data and capture the midpoint timestamp
   * @param reg Starting register
   * @param buf Buffer to fill
   * @param len Number of bytes to read
   * @param timestamp The steady_clock midpoint timestamp populated by this function
   * @return true if successful
   */
  bool read_block_with_timestamp(uint8_t reg, uint8_t* buf, size_t len, std::chrono::steady_clock::time_point& timestamp);

  // Accessors
  int last_errno() const { return last_errno_; }
  const std::string& device() const { return device_; }
  uint8_t address() const { return address_; }

private:
  std::string device_;
  uint8_t address_;
  int fd_;
  int last_errno_;
};

} // namespace core
} // namespace imu_ros2

#endif // IMU_ROS2__CORE__I2C_TRANSPORT_HPP_
