#include "imu_ros2/calibration/accel_calibration.hpp"

namespace imu_ros2 {
namespace calibration {

void AccelCalibration::set_bias(double x, double y, double z)
{
  bias_x_ = x;
  bias_y_ = y;
  bias_z_ = z;
}

void AccelCalibration::apply(core::RawImuData& data) const
{
  data.accel[0] -= bias_x_;
  data.accel[1] -= bias_y_;
  data.accel[2] -= bias_z_;
}

} // namespace calibration
} // namespace imu_ros2
