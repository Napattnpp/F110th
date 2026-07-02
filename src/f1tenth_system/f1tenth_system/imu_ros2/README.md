# imu_ros2

A reusable, unified, and highly optimized IMU framework for F1TENTH.

## Key Features
- **ROS-Independent Core:** The driver logic is completely isolated in a pure C++17 library.
- **Unified Interface:** Supports multiple IMUs (ICM-20948, BMI160) through a simple `DriverFactory`.
- **Zero-Allocation Fast Loop:** Specifically designed for 200–400 Hz output with minimal CPU overhead.
- **Midpoint Timestamping:** Captures `steady_clock` before and after the I²C read transaction for accurate timing.
- **Clean Output:** Publishes strictly `sensor_msgs/Imu` on `/imu/data_raw` (REP-145). Orientation estimation must be performed externally via `imu_filter_madgwick` or `robot_localization`.

## Architecture Rule
> **Rule:** The driver layer (`core` and `drivers`) must **never** include ROS headers (`#include <rclcpp/rclcpp.hpp>`). This enforces strict separation between hardware logic and the ROS application.

## Quick Start

### Build
```bash
colcon build --packages-select imu_ros2
```

### Run
```bash
ros2 launch imu_ros2 imu_launch.py
```

### Calibration
A standalone (ROS-independent) tool is provided to compute the gyroscope bias quickly:
```bash
ros2 run imu_ros2 calibrate_gyro icm20948 /dev/i2c-7 0x68
```
Add the outputs to your YAML file!
