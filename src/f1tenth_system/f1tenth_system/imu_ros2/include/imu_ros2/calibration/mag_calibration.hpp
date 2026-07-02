#ifndef IMU_ROS2__CALIBRATION__MAG_CALIBRATION_HPP_
#define IMU_ROS2__CALIBRATION__MAG_CALIBRATION_HPP_

#include "imu_ros2/core/imu_driver.hpp"

namespace imu_ros2 {
namespace calibration {

class MagCalibration {
public:
  MagCalibration() {
    hard_iron_[0] = 0.0; hard_iron_[1] = 0.0; hard_iron_[2] = 0.0;
    soft_iron_[0] = 1.0; soft_iron_[1] = 0.0; soft_iron_[2] = 0.0;
    soft_iron_[3] = 0.0; soft_iron_[4] = 1.0; soft_iron_[5] = 0.0;
    soft_iron_[6] = 0.0; soft_iron_[7] = 0.0; soft_iron_[8] = 1.0;
  }

  void set_hard_iron(double x, double y, double z);
  void set_soft_iron(const double matrix[9]);
  
  void apply(core::RawImuData& data) const;

private:
  double hard_iron_[3];
  double soft_iron_[9]; // 3x3 matrix in row-major order
};

} // namespace calibration
} // namespace imu_ros2

#endif // IMU_ROS2__CALIBRATION__MAG_CALIBRATION_HPP_
