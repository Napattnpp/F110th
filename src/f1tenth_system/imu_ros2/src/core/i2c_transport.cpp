#include "imu_ros2/core/i2c_transport.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cerrno>
#include <cstring>

namespace imu_ros2 {
namespace core {

I2cTransport::I2cTransport(const std::string& device, uint8_t address)
  : device_(device), address_(address), fd_(-1), last_errno_(0)
{
}

I2cTransport::~I2cTransport()
{
  close_device();
}

bool I2cTransport::open_device()
{
  if (is_open()) {
    return true;
  }

  fd_ = ::open(device_.c_str(), O_RDWR);
  if (fd_ < 0) {
    last_errno_ = errno;
    return false;
  }

  return set_address(address_);
}

void I2cTransport::close_device()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool I2cTransport::is_open() const
{
  return fd_ >= 0;
}

bool I2cTransport::set_address(uint8_t addr)
{
  if (!is_open()) {
    return false;
  }

  if (::ioctl(fd_, I2C_SLAVE, addr) < 0) {
    last_errno_ = errno;
    return false;
  }

  address_ = addr;
  return true;
}

bool I2cTransport::write_byte(uint8_t reg, uint8_t val)
{
  if (!is_open()) {
    return false;
  }

  uint8_t data[2] = {reg, val};
  if (::write(fd_, data, 2) != 2) {
    last_errno_ = errno;
    return false;
  }

  return true;
}

bool I2cTransport::read_byte(uint8_t reg, uint8_t& val)
{
  if (!is_open()) {
    return false;
  }

  if (::write(fd_, &reg, 1) != 1) {
    last_errno_ = errno;
    return false;
  }

  if (::read(fd_, &val, 1) != 1) {
    last_errno_ = errno;
    return false;
  }

  return true;
}

bool I2cTransport::read_block_with_timestamp(uint8_t reg, uint8_t* buf, size_t len, std::chrono::steady_clock::time_point& timestamp)
{
  if (!is_open()) {
    return false;
  }

  if (::write(fd_, &reg, 1) != 1) {
    last_errno_ = errno;
    return false;
  }

  // Midpoint timestamping
  auto t_before = std::chrono::steady_clock::now();
  
  ssize_t bytes_read = ::read(fd_, buf, len);
  
  auto t_after = std::chrono::steady_clock::now();
  timestamp = t_before + (t_after - t_before) / 2;

  if (bytes_read != static_cast<ssize_t>(len)) {
    last_errno_ = errno;
    return false;
  }

  return true;
}

} // namespace core
} // namespace imu_ros2
