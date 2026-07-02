#include "imu_ros2/diagnostics/diagnostics_tracker.hpp"
#include <algorithm>

namespace imu_ros2 {
namespace diagnostics {

DiagnosticsTracker::DiagnosticsTracker()
{
  reset();
}

void DiagnosticsTracker::reset()
{
  total_reads_ = 0;
  total_errors_ = 0;
  last_temperature_ = 0.0;
  min_cycle_time_ms_ = 1e9;
  max_cycle_time_ms_ = 0.0;
  sum_cycle_time_ms_ = 0.0;
  start_time_ = std::chrono::steady_clock::now();
  last_timestamp_ = start_time_;
}

void DiagnosticsTracker::record_read_success(std::chrono::steady_clock::time_point timestamp, double temperature)
{
  total_reads_++;
  last_temperature_ = temperature;

  if (total_reads_ > 1) {
    std::chrono::duration<double, std::milli> dt = timestamp - last_timestamp_;
    double dt_ms = dt.count();
    
    min_cycle_time_ms_ = std::min(min_cycle_time_ms_, dt_ms);
    max_cycle_time_ms_ = std::max(max_cycle_time_ms_, dt_ms);
    sum_cycle_time_ms_ += dt_ms;
  }
  
  last_timestamp_ = timestamp;
}

void DiagnosticsTracker::record_read_failure()
{
  total_errors_++;
}

double DiagnosticsTracker::actual_rate_hz() const
{
  if (total_reads_ < 2) {
    return 0.0;
  }
  std::chrono::duration<double> elapsed = last_timestamp_ - start_time_;
  if (elapsed.count() <= 0.0) return 0.0;
  return static_cast<double>(total_reads_ - 1) / elapsed.count();
}

double DiagnosticsTracker::mean_cycle_time_ms() const
{
  if (total_reads_ < 2) {
    return 0.0;
  }
  return sum_cycle_time_ms_ / static_cast<double>(total_reads_ - 1);
}

} // namespace diagnostics
} // namespace imu_ros2
