import rclpy
from rclpy.node import Node
import numpy as np
import time

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class FTGMapperNode(Node):
    def __init__(self):
        super().__init__('ftg_mapper_node')

        # Reactive Gap Parameters (Mapped from C++ node)
        self.moving_avg_window = 3
        self.max_dist = 8.0 # Look further ahead (was 6.0)
        self.car_width = 0.28
        self.bubble_radius = 0.24
        self.min_obs_dist_threshold = 1.2
        self.disparity_threshold = 0.5
        
        self.gap_target_bias = 0.5
        self.steering_gain = 1.2 # Less aggressive turning (was 1.2)
        
        # Mapping Slow Speeds (Slower than the racing defaults)
        self.vel_close = 0.4
        self.vel_mid = 0.5
        self.vel_far = 0.7
        
        self.depth_close = 1.5
        self.depth_mid = 3.0
        self.depth_far = 4.5
        
        self.fallback_velocity = 0.4
        self.steering_smoothing = 0.6 # Heavy EMA smoothing for stable, gradual turns (was 0.3)
        self.use_constant_speed = True # Set to False to enable dynamic speed scaling

        # Crash Recovery Parameters
        self.recovery_speed = -0.6
        self.recovery_duration = 1.5
        self.crash_distance = 0.25 # meters

        # State
        self.state = 'NORMAL'  # 'NORMAL' or 'RECOVERY'
        self.recovery_start_time = 0.0
        self.prev_steering_angle = 0.0

        # Publishers / Subscribers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.angle_increment = 0.0

        self.get_logger().info("FTG Mapper (C++ Python Port) initialized!")

    def preprocess_lidar(self, ranges, angle_min, angle_increment):
        # Clip ranges, remove nans/infs
        ranges = np.array(ranges)
        ranges = np.nan_to_num(ranges, nan=self.max_dist, posinf=self.max_dist, neginf=0.0)
        ranges = np.clip(ranges, 0.0, self.max_dist)
        
        # Moving average filter
        window = self.moving_avg_window
        if window > 1:
            pad = window // 2
            smoothed = np.zeros_like(ranges)
            for i in range(len(ranges)):
                start_i = max(0, i - pad)
                end_i = min(len(ranges), i + pad + 1)
                smoothed[i] = np.mean(ranges[start_i:end_i])
            ranges = smoothed
        
        # Only look at the front 180 degrees (-90 to +90)
        angles = angle_min + np.arange(len(ranges)) * angle_increment
        valid_indices = np.where((angles >= -np.pi/2) & (angles <= np.pi/2))[0]
        
        return ranges, angles, valid_indices

    def disparity_extending(self, ranges):
        virtual_ranges = np.copy(ranges)
        for i in range(1, len(ranges)):
            diff = ranges[i] - ranges[i-1]
            if abs(diff) > self.disparity_threshold:
                # Find which side is closer
                closer_idx = i - 1 if diff > 0 else i
                closer_dist = ranges[closer_idx]
                
                # Extend the closer distance over the further side by half car width
                theta = np.arcsin(min(1.0, (self.car_width/2.0) / max(0.1, closer_dist)))
                d_index = int(theta / max(1e-5, self.angle_increment))
                
                if diff > 0:
                    # Extend forward (i to i+d_index)
                    start_idx = i
                    end_idx = min(len(ranges), i + d_index)
                    virtual_ranges[start_idx:end_idx] = np.minimum(virtual_ranges[start_idx:end_idx], closer_dist)
                else:
                    # Extend backward (i-d_index to i-1)
                    start_idx = max(0, i - d_index)
                    end_idx = i
                    virtual_ranges[start_idx:end_idx] = np.minimum(virtual_ranges[start_idx:end_idx], closer_dist)
        return virtual_ranges

    def bubble_extend(self, ranges, min_index):
        min_dist = ranges[min_index]
        if min_dist < self.bubble_radius or min_dist >= self.max_dist:
            return
            
        theta = np.arcsin(min(1.0, self.bubble_radius / max(0.1, min_dist)))
        d_index = int(theta / max(1e-5, self.angle_increment))
        
        start_idx = max(0, min_index - d_index)
        end_idx = min(len(ranges), min_index + d_index + 1)
        ranges[start_idx:end_idx] = 0.0

    def find_deepest_valid_gap(self, ranges, valid_indices):
        max_gap_depth = 0.0
        best_start = valid_indices[0]
        best_end = valid_indices[0]
        
        gap_start = -1
        gap_sum = 0.0
        gap_min = 1000.0
        
        for i in valid_indices:
            if ranges[i] > self.min_obs_dist_threshold:
                if gap_start == -1:
                    gap_start = i
                gap_sum += ranges[i]
                gap_min = min(gap_min, ranges[i])
            else:
                if gap_start != -1:
                    gap_length = i - gap_start
                    gap_width = gap_length * gap_min * self.angle_increment
                    if gap_width > 0.8 * self.car_width:
                        mean_depth = gap_sum / gap_length
                        if mean_depth > max_gap_depth:
                            max_gap_depth = mean_depth
                            best_start = gap_start
                            best_end = i
                    gap_start = -1
                    gap_sum = 0.0
                    gap_min = 1000.0
        
        # Check last gap
        if gap_start != -1:
            gap_length = valid_indices[-1] + 1 - gap_start
            gap_width = gap_length * gap_min * self.angle_increment
            if gap_width > 0.8 * self.car_width:
                mean_depth = gap_sum / gap_length
                if mean_depth > max_gap_depth:
                    best_start = gap_start
                    best_end = valid_indices[-1] + 1
                    
        return best_start, best_end

    def find_target_point(self, start_i, end_i, ranges):
        gap_ranges = ranges[start_i:end_i]
        if len(gap_ranges) == 0:
            return (start_i + end_i) // 2
            
        deepest_idx = start_i + np.argmax(gap_ranges)
        center_idx = (start_i + end_i) // 2
        
        # Blend deepest and center
        target_idx = int(0.5 * deepest_idx + 0.5 * center_idx)
        return target_idx

    def calculate_speed(self, steering_angle, forward_clearance):
        if self.use_constant_speed:
            return self.vel_mid

        abs_angle = abs(steering_angle)
        
        if forward_clearance < self.depth_close:
            base_vel = self.vel_close
        elif forward_clearance < self.depth_mid:
            base_vel = self.vel_close + (forward_clearance - self.depth_close) * (self.vel_mid - self.vel_close) / (self.depth_mid - self.depth_close)
        elif forward_clearance < self.depth_far:
            base_vel = self.vel_mid + (forward_clearance - self.depth_mid) * (self.vel_far - self.vel_mid) / (self.depth_far - self.depth_mid)
        else:
            base_vel = self.vel_far
            
        # Angle slowdown (quadratic scaling)
        max_steer_rad = 0.52 # ~30 deg
        angle_ratio = min(1.0, abs_angle / max_steer_rad)
        steer_scale = 1.0 - (angle_ratio ** 2) * (1.0 - 0.5)
        
        final_speed = base_vel * steer_scale
        return max(0.4, min(final_speed, self.vel_far))

    def scan_callback(self, msg):
        self.angle_increment = msg.angle_increment
        ranges, angles, valid_indices = self.preprocess_lidar(msg.ranges, msg.angle_min, msg.angle_increment)
        
        # Check for crash (look directly in front of car, +/- 20 degrees)
        front_indices = np.where((angles >= -np.pi/9) & (angles <= np.pi/9))[0]
        if len(front_indices) > 0:
            min_front_dist = np.min(ranges[front_indices])
            if min_front_dist < self.crash_distance and self.state == 'NORMAL':
                self.get_logger().warn(f"CRASH DETECTED! Distance: {min_front_dist:.2f}m. Entering RECOVERY.")
                self.state = 'RECOVERY'
                self.recovery_start_time = time.time()
                
        # Handle state
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        
        if self.state == 'RECOVERY':
            if time.time() - self.recovery_start_time > self.recovery_duration:
                self.get_logger().info("Recovery complete. Resuming NORMAL driving.")
                self.state = 'NORMAL'
            else:
                drive_msg.drive.speed = self.recovery_speed
                drive_msg.drive.steering_angle = 0.0
                self.drive_pub.publish(drive_msg)
                return
                
        # NORMAL mode (C++ Logic Port)
        
        # 1. Disparity Extending
        virtual_ranges = self.disparity_extending(ranges)
        
        # Find closest point in FOV
        valid_ranges = virtual_ranges[valid_indices]
        if len(valid_ranges) == 0:
            return
            
        min_index = valid_indices[np.argmin(valid_ranges)]
        min_value = virtual_ranges[min_index]
        
        # 2. Bubble Extending
        if min_value < self.max_dist:
            self.bubble_extend(virtual_ranges, min_index)
            
        # 3. Find Deepest Valid Gap
        start_i, end_i = self.find_deepest_valid_gap(virtual_ranges, valid_indices)
        
        gap_width = end_i - start_i
        if gap_width < 5:
            # No valid gap, drive straight slowly
            angle = 0.0
            velocity = self.fallback_velocity
        else:
            # 4. Find Target Point
            best_index = self.find_target_point(start_i, end_i, virtual_ranges)
            angle = angles[best_index] * self.steering_gain
            
            # Calculate forward clearance for speed mapping
            front_idx = np.where((angles >= -np.pi/12) & (angles <= np.pi/12))[0]
            if len(front_idx) > 0:
                forward_clearance = np.mean(virtual_ranges[front_idx])
            else:
                forward_clearance = self.max_dist
                
            # 5. Calculate Dynamic Speed
            velocity = self.calculate_speed(angle, forward_clearance)
            
        # 6. Apply Steering Smoothing (EMA)
        angle = self.prev_steering_angle * self.steering_smoothing + angle * (1.0 - self.steering_smoothing)
        self.prev_steering_angle = angle
        
        # Publish
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = angle
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FTGMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
