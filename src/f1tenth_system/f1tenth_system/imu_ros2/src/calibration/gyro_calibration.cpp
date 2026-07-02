#include "imu_ros2/calibration/gyro_calibration.hpp"

namespace imu_ros2 {
namespace calibration {

void GyroCalibration::set_bias(double x, double y, double z)
{
  bias_x_ = x;
  bias_y_ = y;
  bias_z_ = z;
}

void GyroCalibration::apply(core::RawImuData& data) const
{
  data.gyro[0] -= bias_x_;
  data.gyro[1] -= bias_y_;
  data.gyro[2] -= bias_z_;
}

} // namespace calibration
} // namespace imu_ros2
