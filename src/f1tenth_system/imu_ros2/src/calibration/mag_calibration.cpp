#include "imu_ros2/calibration/mag_calibration.hpp"
#include <cstring>

namespace imu_ros2 {
namespace calibration {

void MagCalibration::set_hard_iron(double x, double y, double z)
{
  hard_iron_[0] = x;
  hard_iron_[1] = y;
  hard_iron_[2] = z;
}

void MagCalibration::set_soft_iron(const double matrix[9])
{
  std::memcpy(soft_iron_, matrix, 9 * sizeof(double));
}

void MagCalibration::apply(core::RawImuData& data) const
{
  if (!data.has_mag) {
    return;
  }

  double x = data.mag[0] - hard_iron_[0];
  double y = data.mag[1] - hard_iron_[1];
  double z = data.mag[2] - hard_iron_[2];

  data.mag[0] = soft_iron_[0] * x + soft_iron_[1] * y + soft_iron_[2] * z;
  data.mag[1] = soft_iron_[3] * x + soft_iron_[4] * y + soft_iron_[5] * z;
  data.mag[2] = soft_iron_[6] * x + soft_iron_[7] * y + soft_iron_[8] * z;
}

} // namespace calibration
} // namespace imu_ros2
