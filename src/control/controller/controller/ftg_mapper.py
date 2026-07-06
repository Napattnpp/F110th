import rclpy
from rclpy.node import Node
import numpy as np
import time

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class FTGMapperNode(Node):
    def __init__(self):
        super().__init__('ftg_mapper_node')

        # Subscribe to LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publish to /drive
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        # Visualization publishers
        self.virtual_scan_pub = self.create_publisher(LaserScan, '/virtual_scan', 10)
        self.marker_pub = self.create_publisher(Marker, '/steering_marker', 10)
        self.gap_marker_pub = self.create_publisher(Marker, '/gap_boundaries', 10)
        self.target_pub = self.create_publisher(Marker, '/target_point', 10)
        self.bubble_pub = self.create_publisher(Marker, '/safety_bubble', 10)
        self.mode_text_pub = self.create_publisher(Marker, '/mode_text_marker', 10)
        self.forward_cone_pub = self.create_publisher(Marker, '/forward_cone_visual', 10)

        # Robot physical parameters
        self.car_width = 0.25
        self.wheelbase = 0.295
        self.max_steering_angle = 0.42

        # Safety parameters
        self.bubble_radius = 0.38
        self.min_wall_distance = 0.25
        self.declare_parameter('disparity_threshold', 0.5)

        # ===== SPEED PARAMETERS (m/s) =====
        self.declare_parameter('max_speed', 3.0)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('cruise_speed', 1.5)
        self.declare_parameter('slowing_distance', 4.0)
        self.declare_parameter('emergency_distance', 1.5)
        self.declare_parameter('use_constant_speed', True)
        self.declare_parameter('constant_speed', 0.5)

        # ===== FORWARD DETECTION PARAMETERS =====
        self.declare_parameter('forward_cone_half_angle_deg', 20.0)

        # ===== ADAPTIVE MODE PARAMETERS =====
        self.declare_parameter('direct_mode_threshold', 4.5)
        self.declare_parameter('gap_mode_threshold', 3.0)

        # ===== BALANCED HEADING WEIGHTS =====
        self.declare_parameter('heading_bias_max', 0.2)
        self.declare_parameter('heading_bias_min', 0.0)

        # ===== CENTER GAP PREFERENCE =====
        self.declare_parameter('center_penalty_coefficient', 1.5)

        # ===== TARGET POINT WEIGHTS (ADAPTIVE) =====
        self.declare_parameter('gap_deepest_weight_max', 0.2)
        self.declare_parameter('gap_deepest_weight_min', 0.0)

        # ===== STEERING CONTROL =====
        self.declare_parameter('steering_gain', 1.8)
        self.declare_parameter('steering_filter_alpha', 1.8)

        # ===== CRASH RECOVERY PARAMETERS =====
        self.declare_parameter('recovery_speed', -0.6)
        self.declare_parameter('recovery_duration', 1.0)
        self.declare_parameter('crash_distance', 0.25)

        # ===== VISUALIZATION PARAMETER =====
        self.declare_parameter('visualize', True)

        # State tracking
        self.state = 'NORMAL'  # 'NORMAL' or 'RECOVERY'
        self.recovery_start_time = 0.0
        self.prev_steering = 0.0
        self.prev_speed = 0.0
        self.scan_count = 0

        self.get_logger().info("Optimized FTG Mapper Node initialized with visualizations and toggles!")

    def preprocess_scan(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        ranges = np.nan_to_num(ranges, nan=scan_msg.range_max,
                               posinf=scan_msg.range_max, neginf=0.0)
        return ranges

    def bubble_extend(self, ranges, center_index, angle_increment, extend_value=False):
        if ranges[center_index] < self.bubble_radius:
            return ranges
            
        dtheta = np.arcsin(min(1.0, self.bubble_radius / max(0.1, ranges[center_index])))
        d_index = int(dtheta / max(1e-5, angle_increment))
        
        start_idx = max(0, center_index - d_index)
        end_idx = min(len(ranges), center_index + d_index + 1)
        
        if extend_value:
            ranges[start_idx:end_idx] = np.minimum(ranges[start_idx:end_idx], ranges[center_index])
        else:
            ranges[start_idx:end_idx] = 0.0
            
        return ranges

    def disparity_extending(self, ranges, angle_increment, angle_min, disparity_threshold):
        virtual_ranges = np.copy(ranges)
        
        # Only look at the front 180 degrees (-90 to +90)
        angles = angle_min + np.arange(len(ranges)) * angle_increment
        valid_indices = np.where((angles >= -np.pi/2) & (angles <= np.pi/2))[0]
        
        if len(valid_indices) < 2:
            return virtual_ranges
            
        start_idx = valid_indices[0]
        end_idx = valid_indices[-1]
        
        for i in range(start_idx + 1, end_idx + 1):
            diff = ranges[i] - ranges[i-1]
            if abs(diff) > disparity_threshold:
                if ranges[i] < ranges[i-1]:
                    virtual_ranges = self.bubble_extend(virtual_ranges, i, angle_increment, extend_value=True)
                else:
                    virtual_ranges = self.bubble_extend(virtual_ranges, i-1, angle_increment, extend_value=True)
        return virtual_ranges

    def get_forward_clearance(self, ranges, angle_min, angle_increment, min_wall_distance, emergency_distance):
        num_ranges = len(ranges)
        mid_idx = num_ranges // 2

        # Use forward cone half angle parameter in degrees
        half_angle_deg = self.get_parameter('forward_cone_half_angle_deg').value
        cone_angle_rad = np.radians(half_angle_deg)
        cone_indices = int(cone_angle_rad / max(1e-5, angle_increment))

        # Get forward cone ranges
        start_idx = max(0, mid_idx - cone_indices)
        end_idx = min(num_ranges, mid_idx + cone_indices)

        forward_ranges = ranges[start_idx:end_idx]

        # Check for very close obstacles first
        very_close = forward_ranges[(forward_ranges > 0.0) & (forward_ranges < min_wall_distance)]
        if len(very_close) > 0:
            return 0.1  # EMERGENCY!

        # Filter valid ranges
        clipped_ranges = np.clip(forward_ranges, min_wall_distance, 30.0)
        valid_ranges = clipped_ranges[clipped_ranges > min_wall_distance]

        if len(valid_ranges) == 0:
            return 10.0

        return float(np.min(valid_ranges))

    def create_safety_bubble(self, ranges, angle_increment, min_wall_distance, bubble_radius):
        ranges_clipped = np.clip(ranges, min_wall_distance, 30.0)

        close_threshold = 1.5
        close_indices = np.where(ranges_clipped < close_threshold)[0]

        if len(close_indices) == 0:
            return ranges_clipped

        bubble_copy = ranges_clipped.copy()

        for idx in close_indices:
            obstacle_distance = max(ranges_clipped[idx], 0.1)
            angular_bubble = bubble_radius / obstacle_distance
            bubble_indices = int(np.ceil(angular_bubble / max(1e-5, angle_increment)))

            min_idx = max(0, idx - bubble_indices)
            max_idx = min(len(ranges_clipped), idx + bubble_indices + 1)
            bubble_copy[min_idx:max_idx] = 0.0

        return bubble_copy

    def calculate_mode_weight(self, min_obstacle_distance, direct_mode_threshold, gap_mode_threshold,
                              heading_bias_min, heading_bias_max, gap_deepest_weight_min, gap_deepest_weight_max):
        if min_obstacle_distance >= direct_mode_threshold:
            mode_factor = 1.0
            mode_name = "DIRECT"
        elif min_obstacle_distance <= gap_mode_threshold:
            mode_factor = 0.0
            mode_name = "GAP"
        else:
            range_width = direct_mode_threshold - gap_mode_threshold
            mode_factor = (min_obstacle_distance - gap_mode_threshold) / range_width
            mode_name = "TRANSITION"

        heading_bias = (heading_bias_min + mode_factor * (heading_bias_max - heading_bias_min))
        deepest_weight = (gap_deepest_weight_max - mode_factor * (gap_deepest_weight_max - gap_deepest_weight_min))
        center_weight = 1.0 - deepest_weight

        return heading_bias, deepest_weight, center_weight, mode_name

    def find_largest_gap(self, ranges, center_penalty_coefficient):
        threshold = 0.5
        mask = ranges > threshold

        gaps = []
        in_gap = False
        gap_start = 0

        for i in range(len(mask)):
            if mask[i] and not in_gap:
                gap_start = i
                in_gap = True
            elif not mask[i] and in_gap:
                gap_width = i - gap_start
                if gap_width > 5:
                    gaps.append((gap_start, i, gap_width))
                in_gap = False

        if in_gap:
            gap_width = len(mask) - gap_start
            if gap_width > 5:
                gaps.append((gap_start, len(mask), gap_width))

        if not gaps:
            mid = len(ranges) // 2
            return mid - 10, mid + 10

        best_score = -float('inf')
        best_gap = None
        num_ranges = len(ranges)
        mid_idx = num_ranges // 2

        for gap_start, gap_end, gap_width in gaps:
            gap_center_idx = (gap_start + gap_end) // 2
            distance_from_center = abs(gap_center_idx - mid_idx)

            score = gap_width - center_penalty_coefficient * distance_from_center

            if score > best_score:
                best_score = score
                best_gap = (gap_start, gap_end, gap_width)

        if best_gap:
            return best_gap[0], best_gap[1]
        else:
            mid = len(ranges) // 2
            return mid - 10, mid + 10

    def find_target_point(self, ranges, gap_start, gap_end, deepest_weight, center_weight):
        if gap_start >= gap_end or gap_end - gap_start < 3:
            return (gap_start + gap_end) // 2

        gap_ranges = ranges[gap_start:gap_end]
        deepest_idx = np.argmax(gap_ranges)
        gap_center_idx = len(gap_ranges) // 2

        target_idx = int(deepest_weight * deepest_idx + center_weight * gap_center_idx)
        return gap_start + target_idx

    def apply_speed_dependent_damping(self, steering_angle, current_speed):
        speed_ratio = min(current_speed / 3.0, 3.0)
        damping_factor = max(0.6, 1.0 - 0.12 * speed_ratio)
        return steering_angle * damping_factor

    def calculate_steering(self, target_idx, num_ranges, angle_min, angle_increment,
                           current_speed, heading_bias, steering_gain, steering_filter_alpha):
        target_angle_gap = angle_min + target_idx * angle_increment
        target_angle_heading = 0.0

        W_gap = 1.0 - heading_bias
        blended_angle = W_gap * target_angle_gap + heading_bias * target_angle_heading

        raw_steering = blended_angle * steering_gain
        steering = np.clip(raw_steering, -self.max_steering_angle, self.max_steering_angle)

        steering = self.apply_speed_dependent_damping(steering, current_speed)

        filtered_steering = (steering_filter_alpha * steering +
                             (1 - steering_filter_alpha) * self.prev_steering)
        self.prev_steering = filtered_steering

        return float(filtered_steering)

    def calculate_speed(self, steering_angle, target_distance, gap_width_idx, forward_clearance,
                        min_speed, cruise_speed, max_speed, emergency_distance, slowing_distance):
        abs_angle = abs(steering_angle)

        # Base speed from steering angle
        if abs_angle > 0.35:
            base_speed = min_speed
        elif abs_angle > 0.25:
            ratio = (0.35 - abs_angle) / 0.10
            base_speed = min_speed + ratio * 0.3 * (cruise_speed - min_speed)
        elif abs_angle > 0.15:
            ratio = (0.25 - abs_angle) / 0.10
            base_speed = 0.3 * cruise_speed + ratio * 0.4 * (cruise_speed - 0.3 * cruise_speed)
        elif abs_angle > 0.08:
            ratio = (0.15 - abs_angle) / 0.07
            base_speed = 0.7 * cruise_speed + ratio * 0.3 * (cruise_speed - 0.7 * cruise_speed)
        else:
            ratio = (0.08 - abs_angle) / 0.08
            base_speed = cruise_speed + ratio * (max_speed - cruise_speed)

        # Distance-based scaling using FORWARD CLEARANCE
        if forward_clearance < emergency_distance:
            distance_factor = 0.15 + (forward_clearance / emergency_distance) * 0.05
            distance_factor = max(0.05, distance_factor)
        elif forward_clearance < slowing_distance:
            scale_range = slowing_distance - emergency_distance
            scale_progress = (forward_clearance - emergency_distance) / scale_range
            distance_factor = 0.15 + 0.85 * scale_progress
        else:
            distance_factor = 1.0

        base_speed *= distance_factor

        gap_width_factor = min(1.0, gap_width_idx / 60.0)
        base_speed *= (0.6 + 0.4 * gap_width_factor)

        final_speed = max(0.5, min(base_speed, max_speed))
        return float(final_speed)

    def get_minimum_obstacle_distance(self, ranges, min_wall_distance):
        very_close = ranges[(ranges > 0.0) & (ranges < min_wall_distance)]
        if len(very_close) > 0:
            return 0.1

        clipped_ranges = np.clip(ranges, min_wall_distance, 30.0)
        valid_ranges = clipped_ranges[clipped_ranges > min_wall_distance]

        if len(valid_ranges) == 0:
            return 10.0

        return float(np.min(valid_ranges))

    # ========================================== #
    #           VISUALIZATION HELPERS            #
    # ========================================== #
    def publish_virtual_scan(self, scan_msg, virtual_ranges):
        virtual_scan = LaserScan()
        virtual_scan.header = scan_msg.header
        virtual_scan.angle_min = scan_msg.angle_min
        virtual_scan.angle_max = scan_msg.angle_max
        virtual_scan.angle_increment = scan_msg.angle_increment
        virtual_scan.time_increment = scan_msg.time_increment
        virtual_scan.scan_time = scan_msg.scan_time
        virtual_scan.range_min = scan_msg.range_min
        virtual_scan.range_max = scan_msg.range_max
        virtual_scan.ranges = [float(r) for r in virtual_ranges]
        self.virtual_scan_pub.publish(virtual_scan)

    def publish_steering_marker(self, scan_msg, steering_angle, speed):
        marker = Marker()
        marker.header.frame_id = scan_msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "steering_target"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        start_point = Point()
        start_point.x = 0.0
        start_point.y = 0.0
        start_point.z = 0.0

        arrow_length = 2.0
        end_point = Point()
        end_point.x = arrow_length * np.cos(steering_angle)
        end_point.y = arrow_length * np.sin(steering_angle)
        end_point.z = 0.0

        marker.points.append(start_point)
        marker.points.append(end_point)

        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0

        if speed >= 3.0:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0  # Green
        elif speed >= 1.5:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0  # Yellow
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0  # Red

        self.marker_pub.publish(marker)

    def publish_gap_boundaries(self, scan_msg, gap_start_i, gap_end_i, virtual_ranges):
        marker = Marker()
        marker.header.frame_id = scan_msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "gap_boundaries"
        marker.id = 1
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        start_angle = scan_msg.angle_min + (gap_start_i * scan_msg.angle_increment)
        start_dist = virtual_ranges[gap_start_i]
        p1 = Point()
        p1.x = start_dist * np.cos(start_angle)
        p1.y = start_dist * np.sin(start_angle)
        p1.z = 0.0

        # Boundary check
        gap_end_i = min(len(virtual_ranges) - 1, gap_end_i)
        end_angle = scan_msg.angle_min + (gap_end_i * scan_msg.angle_increment)
        end_dist = virtual_ranges[gap_end_i]
        p2 = Point()
        p2.x = end_dist * np.cos(end_angle)
        p2.y = end_dist * np.sin(end_angle)
        p2.z = 0.0

        marker.points.append(p1)
        marker.points.append(p2)

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.8
        marker.color.b = 0.3

        self.gap_marker_pub.publish(marker)

    def publish_target_point(self, scan_msg, target_index, virtual_ranges):
        marker = Marker()
        marker.header.frame_id = scan_msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_point"
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        target_angle = scan_msg.angle_min + (target_index * scan_msg.angle_increment)
        target_dist = virtual_ranges[target_index]

        marker.pose.position.x = target_dist * np.cos(target_angle)
        marker.pose.position.y = target_dist * np.sin(target_angle)
        marker.pose.position.z = 0.0

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.target_pub.publish(marker)

    def publish_bubble(self, scan_msg, center_index, distance):
        marker = Marker()
        marker.header.frame_id = scan_msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "safety_bubble"
        marker.id = 3
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        angle = scan_msg.angle_min + (center_index * scan_msg.angle_increment)
        marker.pose.position.x = distance * np.cos(angle)
        marker.pose.position.y = distance * np.sin(angle)
        marker.pose.position.z = 0.0

        marker.scale.x = self.bubble_radius * 2.0
        marker.scale.y = self.bubble_radius * 2.0
        marker.scale.z = 0.05

        marker.color.a = 0.4
        marker.color.r = 1.0
        marker.color.g = 0.3
        marker.color.b = 0.0

        self.bubble_pub.publish(marker)

    def publish_mode_text(self, scan_msg, mode_name, state):
        marker = Marker()
        marker.header.frame_id = scan_msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mode_text"
        marker.id = 4
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0  # 1 meter above the car
        
        marker.scale.z = 0.4  # Text height
        
        if state == 'RECOVERY':
            marker.text = "RECOVERY"
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
        else:
            marker.text = f"Mode: {mode_name}"
            if mode_name == "DIRECT":
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif mode_name == "GAP":
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
            else:  # TRANSITION
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            marker.color.a = 1.0
            
        self.mode_text_pub.publish(marker)

    def publish_forward_cone(self, scan_msg, half_angle_deg, slowing_distance):
        marker = Marker()
        marker.header.frame_id = scan_msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "forward_cone"
        marker.id = 5
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        marker.scale.x = 0.03  # Line width
        marker.color.a = 0.6
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0  # Light blue
        
        # Calculate points
        rad = np.radians(half_angle_deg)
        
        p_origin = Point()
        p_origin.x = 0.0
        p_origin.y = 0.0
        p_origin.z = 0.0
        
        p_left = Point()
        p_left.x = slowing_distance * np.cos(rad)
        p_left.y = slowing_distance * np.sin(rad)
        p_left.z = 0.0
        
        p_right = Point()
        p_right.x = slowing_distance * np.cos(-rad)
        p_right.y = slowing_distance * np.sin(-rad)
        p_right.z = 0.0
        
        # Draw a wedge: origin -> left -> right -> origin
        marker.points.append(p_origin)
        marker.points.append(p_left)
        marker.points.append(p_right)
        marker.points.append(p_origin)
        
        self.forward_cone_pub.publish(marker)

    def scan_callback(self, msg):
        self.scan_count += 1

        # Fetch parameter values
        use_constant_speed = self.get_parameter('use_constant_speed').value
        constant_speed = self.get_parameter('constant_speed').value
        max_speed = self.get_parameter('max_speed').value
        min_speed = self.get_parameter('min_speed').value
        cruise_speed = self.get_parameter('cruise_speed').value
        slowing_distance = self.get_parameter('slowing_distance').value
        emergency_distance = self.get_parameter('emergency_distance').value
        direct_mode_threshold = self.get_parameter('direct_mode_threshold').value
        gap_mode_threshold = self.get_parameter('gap_mode_threshold').value
        heading_bias_max = self.get_parameter('heading_bias_max').value
        heading_bias_min = self.get_parameter('heading_bias_min').value
        center_penalty_coefficient = self.get_parameter('center_penalty_coefficient').value
        gap_deepest_weight_max = self.get_parameter('gap_deepest_weight_max').value
        gap_deepest_weight_min = self.get_parameter('gap_deepest_weight_min').value
        steering_gain = self.get_parameter('steering_gain').value
        steering_filter_alpha = self.get_parameter('steering_filter_alpha').value
        recovery_speed = self.get_parameter('recovery_speed').value
        recovery_duration = self.get_parameter('recovery_duration').value
        crash_distance = self.get_parameter('crash_distance').value
        visualize = self.get_parameter('visualize').value
        disparity_threshold = self.get_parameter('disparity_threshold').value

        # Process LiDAR (Preprocess + Disparity Extend)
        raw_ranges = self.preprocess_scan(msg)
        ranges = self.disparity_extending(raw_ranges, msg.angle_increment, msg.angle_min, disparity_threshold)

        # Check for crash (look directly in front of car, +/- 20 degrees)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        front_indices = np.where((angles >= -np.pi/9) & (angles <= np.pi/9))[0]
        if len(front_indices) > 0:
            min_front_dist = np.min(ranges[front_indices])
            if min_front_dist < crash_distance and self.state == 'NORMAL':
                self.get_logger().warn(f"CRASH DETECTED! Distance: {min_front_dist:.2f}m. Entering RECOVERY.")
                self.state = 'RECOVERY'
                self.recovery_start_time = time.time()

        # Handle State Machine
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()

        if self.state == 'RECOVERY':
            if time.time() - self.recovery_start_time > recovery_duration:
                self.get_logger().info("Recovery complete. Resuming NORMAL driving.")
                self.state = 'NORMAL'
            else:
                drive_msg.drive.speed = recovery_speed
                drive_msg.drive.steering_angle = 0.0
                self.drive_pub.publish(drive_msg)
                
                # Publish recovery text marker if enabled
                if visualize:
                    self.publish_mode_text(msg, "GAP", self.state)
                return

        # NORMAL mode (Balanced Follow Gap Logic)
        min_obstacle_distance = self.get_minimum_obstacle_distance(ranges, self.min_wall_distance)
        forward_clearance = self.get_forward_clearance(
            ranges, msg.angle_min, msg.angle_increment, self.min_wall_distance, emergency_distance)

        # Mode weights
        heading_bias, deepest_weight, center_weight, mode_name = self.calculate_mode_weight(
            min_obstacle_distance, direct_mode_threshold, gap_mode_threshold,
            heading_bias_min, heading_bias_max, gap_deepest_weight_min, gap_deepest_weight_max)

        # Safety bubble
        safe_ranges = self.create_safety_bubble(ranges, msg.angle_increment, self.min_wall_distance, self.bubble_radius)

        # Find gap
        gap_start, gap_end = self.find_largest_gap(safe_ranges, center_penalty_coefficient)
        gap_width = gap_end - gap_start

        # Find target point
        target_idx = self.find_target_point(safe_ranges, gap_start, gap_end, deepest_weight, center_weight)
        target_distance = safe_ranges[target_idx]

        # Calculate steering
        steering_angle = self.calculate_steering(
            target_idx, len(safe_ranges), msg.angle_min, msg.angle_increment,
            self.prev_speed, heading_bias, steering_gain, steering_filter_alpha)

        # Calculate speed
        if use_constant_speed:
            speed = constant_speed
        else:
            speed = self.calculate_speed(
                steering_angle, target_distance, gap_width, forward_clearance,
                min_speed, cruise_speed, max_speed, emergency_distance, slowing_distance)

        self.prev_speed = speed

        # Sanity check for steering_angle and speed (prevent NaN/Inf and ensure float range)
        if not np.isfinite(steering_angle):
            steering_angle = 0.0
        else:
            steering_angle = float(np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle))

        if not np.isfinite(speed):
            speed = 0.0
        else:
            speed = float(speed)

        # Publish ackermann drive message
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)

        # Publish visualizations if enabled
        if visualize:
            self.publish_virtual_scan(msg, safe_ranges)
            self.publish_steering_marker(msg, steering_angle, speed)
            self.publish_mode_text(msg, mode_name, self.state)
            self.publish_forward_cone(msg, self.get_parameter('forward_cone_half_angle_deg').value, slowing_distance)
            
            if gap_width >= 5:
                self.publish_gap_boundaries(msg, gap_start, gap_end, safe_ranges)
                self.publish_target_point(msg, target_idx, safe_ranges)
            
            # Find closest point in front 180 degrees for safety bubble visualization
            valid_indices = np.where((angles >= -np.pi/2) & (angles <= np.pi/2))[0]
            if len(valid_indices) > 0:
                min_idx = valid_indices[np.argmin(ranges[valid_indices])]
                min_val = ranges[min_idx]
                if min_val < 30.0: # Keep it within valid LiDAR range
                    self.publish_bubble(msg, min_idx, min_val)


def main(args=None):
    rclpy.init(args=args)
    node = FTGMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
