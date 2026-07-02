#ifndef IMU_ROS2__CORE__IMU_DRIVER_HPP_
#define IMU_ROS2__CORE__IMU_DRIVER_HPP_

#include <chrono>
#include <cstdint>
#include <string>

namespace imu_ros2 {
namespace core {

/**
 * @brief Raw IMU data (ROS-independent)
 */
struct RawImuData {
  std::chrono::steady_clock::time_point timestamp;

  double accel[3]; // m/s^2 (x, y, z)
  double gyro[3];  // rad/s (x, y, z)
  double mag[3];   // microTesla (x, y, z)

  double temperature; // Celsius

  bool has_mag;
  bool valid;
};

/**
 * @brief Simple state machine for the driver
 */
enum class DriverState {
  DISCONNECTED,
  INITIALIZING,
  CALIBRATING,
  RUNNING,
  ERROR
};

/**
 * @brief Abstract interface for an IMU sensor driver.
 * Must NOT contain any ROS dependencies.
 */
class ImuDriver {
public:
  virtual ~ImuDriver() = default;

  /**
   * @brief Initialize the sensor
   * @return true if successful
   */
  virtual bool initialize() = 0;

  /**
   * @brief Attempt to reconnect to the sensor
   * @return true if successful
   */
  virtual bool reconnect() = 0;

  /**
   * @brief Read raw data from the sensor
   * @param data Struct to populate
   * @return true if successful
   */
  virtual bool read_data(RawImuData& data) = 0;

  /**
   * @brief Set the accelerometer full-scale range
   * @param range_g Range in g (e.g., 2, 4, 8, 16)
   * @return true if successful
   */
  virtual bool set_accel_range(int range_g) = 0;

  /**
   * @brief Set the gyroscope full-scale range
   * @param range_dps Range in degrees per second (e.g., 125, 250, 500, 1000, 2000)
   * @return true if successful
   */
  virtual bool set_gyro_range(int range_dps) = 0;

  /**
   * @brief Set the output data rate
   * @param hz Rate in Hz
   * @return true if successful
   */
  virtual bool set_output_data_rate(int hz) = 0;

  /**
   * @brief Does this sensor have a magnetometer?
   */
  virtual bool has_magnetometer() const = 0;

  /**
   * @brief Get the sensor's name
   */
  virtual std::string sensor_name() const = 0;

  /**
   * @brief Get the current state
   */
  virtual DriverState get_state() const = 0;

  /**
   * @brief Get total consecutive read errors
   */
  virtual uint32_t get_consecutive_errors() const = 0;
};

} // namespace core
} // namespace imu_ros2

#endif // IMU_ROS2__CORE__IMU_DRIVER_HPP_
