#include <iostream>
#include <chrono>
#include <thread>
#include <vector>

#include "imu_ros2/core/driver_factory.hpp"
#include "imu_ros2/core/imu_driver.hpp"

using namespace imu_ros2;

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0] << " <sensor_type> <i2c_device> <i2c_addr_hex>\n";
    std::cerr << "Example: " << argv[0] << " icm20948 /dev/i2c-7 0x68\n";
    return 1;
  }

  std::string sensor_type = argv[1];
  std::string i2c_device = argv[2];
  uint8_t i2c_addr = static_cast<uint8_t>(std::stoul(argv[3], nullptr, 16));

  auto driver = core::DriverFactory::create(sensor_type, i2c_device, i2c_addr);
  if (!driver) {
    std::cerr << "Failed to create driver for type: " << sensor_type << "\n";
    return 1;
  }

  std::cout << "Initializing " << sensor_type << " on " << i2c_device << " at 0x" << std::hex << (int)i2c_addr << std::dec << "...\n";
  if (!driver->initialize()) {
    std::cerr << "Initialization failed!\n";
    return 1;
  }
  
  std::cout << "Initialization successful.\n";
  
  // Set ranges for best resolution during calibration
  driver->set_gyro_range(250); // +/- 250 dps

  std::cout << "Please keep the IMU perfectly still.\n";
  std::cout << "Starting calibration in 3 seconds...\n";
  std::this_thread::sleep_for(std::chrono::seconds(3));
  
  std::cout << "Calibrating...\n";

  const int num_samples = 1000;
  double sum_x = 0;
  double sum_y = 0;
  double sum_z = 0;
  int successful_samples = 0;

  for (int i = 0; i < num_samples; ++i) {
    core::RawImuData data;
    if (driver->read_data(data) && data.valid) {
      sum_x += data.gyro[0];
      sum_y += data.gyro[1];
      sum_z += data.gyro[2];
      successful_samples++;
    } else {
      std::cerr << "Read failed at sample " << i << "\n";
    }

    if (i % 100 == 0) {
      std::cout << "Progress: " << i << "/" << num_samples << "\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5)); // ~200Hz
  }

  if (successful_samples == 0) {
    std::cerr << "Failed to read any samples.\n";
    return 1;
  }

  double bias_x = sum_x / successful_samples;
  double bias_y = sum_y / successful_samples;
  double bias_z = sum_z / successful_samples;

  std::cout << "\nCalibration Complete!\n";
  std::cout << "Total samples: " << successful_samples << "\n\n";

  std::cout << "Add these values to your ROS 2 params YAML file:\n\n";
  std::cout << "gyro_bias_x: " << bias_x << "\n";
  std::cout << "gyro_bias_y: " << bias_y << "\n";
  std::cout << "gyro_bias_z: " << bias_z << "\n";

  return 0;
}
