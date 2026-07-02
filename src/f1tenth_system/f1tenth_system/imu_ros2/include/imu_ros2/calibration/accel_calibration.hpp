#ifndef IMU_ROS2__CALIBRATION__ACCEL_CALIBRATION_HPP_
#define IMU_ROS2__CALIBRATION__ACCEL_CALIBRATION_HPP_

#include "imu_ros2/core/imu_driver.hpp"

namespace imu_ros2 {
namespace calibration {

class AccelCalibration {
public:
  AccelCalibration() : bias_x_(0.0), bias_y_(0.0), bias_z_(0.0) {}

  void set_bias(double x, double y, double z);
  void apply(core::RawImuData& data) const;

  double x() const { return bias_x_; }
  double y() const { return bias_y_; }
  double z() const { return bias_z_; }

private:
  double bias_x_;
  double bias_y_;
  double bias_z_;
};

} // namespace calibration
} // namespace imu_ros2

#endif // IMU_ROS2__CALIBRATION__ACCEL_CALIBRATION_HPP_
