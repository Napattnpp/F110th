import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import sys

class ImuCovarianceCalculator(Node):
    def __init__(self):
        super().__init__('imu_cov_calculator')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.listener_callback,
            10)
        self.accel_data = {'x': [], 'y': [], 'z': []}
        self.gyro_data = {'x': [], 'y': [], 'z': []}
        self.sample_count = 0
        self.target_samples = 500
        self.get_logger().info('Waiting for IMU data on /imu/data_raw... Keep the robot perfectly still.')

    def listener_callback(self, msg):
        self.accel_data['x'].append(msg.linear_acceleration.x)
        self.accel_data['y'].append(msg.linear_acceleration.y)
        self.accel_data['z'].append(msg.linear_acceleration.z)
        
        self.gyro_data['x'].append(msg.angular_velocity.x)
        self.gyro_data['y'].append(msg.angular_velocity.y)
        self.gyro_data['z'].append(msg.angular_velocity.z)
        
        self.sample_count += 1
        if self.sample_count % 100 == 0:
            self.get_logger().info(f'Collected {self.sample_count}/{self.target_samples} samples...')
            
        if self.sample_count >= self.target_samples:
            self.calculate_and_print()
            sys.exit(0)

    def calculate_and_print(self):
        accel_vars = [
            np.var(self.accel_data['x']),
            np.var(self.accel_data['y']),
            np.var(self.accel_data['z'])
        ]
        
        gyro_vars = [
            np.var(self.gyro_data['x']),
            np.var(self.gyro_data['y']),
            np.var(self.gyro_data['z'])
        ]
        
        print("\n--- Calibration Complete ---")
        print(f"Accel Covariance: [{accel_vars[0]:.8f}, {accel_vars[1]:.8f}, {accel_vars[2]:.8f}]")
        print(f"Gyro Covariance:  [{gyro_vars[0]:.8f}, {gyro_vars[1]:.8f}, {gyro_vars[2]:.8f}]")
        
def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceCalculator()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
