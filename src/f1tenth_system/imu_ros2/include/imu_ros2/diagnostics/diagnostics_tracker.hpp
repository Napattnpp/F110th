#ifndef IMU_ROS2__DIAGNOSTICS__DIAGNOSTICS_TRACKER_HPP_
#define IMU_ROS2__DIAGNOSTICS__DIAGNOSTICS_TRACKER_HPP_

#include <cstdint>
#include <chrono>

namespace imu_ros2 {
namespace diagnostics {

class DiagnosticsTracker {
public:
  DiagnosticsTracker();

  void record_read_success(std::chrono::steady_clock::time_point timestamp, double temperature);
  void record_read_failure();
  void reset();

  // Metrics
  double actual_rate_hz() const;
  uint64_t total_reads() const { return total_reads_; }
  uint64_t total_errors() const { return total_errors_; }
  double last_temperature() const { return last_temperature_; }
  
  // Timing stats (in milliseconds)
  double min_cycle_time_ms() const { return min_cycle_time_ms_; }
  double max_cycle_time_ms() const { return max_cycle_time_ms_; }
  double mean_cycle_time_ms() const;

private:
  uint64_t total_reads_;
  uint64_t total_errors_;
  double last_temperature_;

  std::chrono::steady_clock::time_point last_timestamp_;
  std::chrono::steady_clock::time_point start_time_;
  
  double min_cycle_time_ms_;
  double max_cycle_time_ms_;
  double sum_cycle_time_ms_;
};

} // namespace diagnostics
} // namespace imu_ros2

#endif // IMU_ROS2__DIAGNOSTICS__DIAGNOSTICS_TRACKER_HPP_
