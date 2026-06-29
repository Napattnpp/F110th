#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

class IDController(Node):
    
    def __init__(self):
        super().__init__('id_controller')
        
        # Publishers
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, 'drive_topic', 10)
        self.erpm_pub = self.create_publisher(Float64, '/vesc/commands/motor/speed', 10)
        self.servo_pub = self.create_publisher(Float64, '/vesc/commands/servo/position', 10)
        self.current_pub = self.create_publisher(Float64, '/vesc/commands/motor/current', 10)
        self.brake_pub = self.create_publisher(Float64, '/vesc/commands/motor/brake', 10)

        # Declare parameters with default values
        self.declare_parameter('experiment', 1)
        self.declare_parameter('const_erpm', 0.0)
        self.declare_parameter('const_servo', 0.5)
        self.declare_parameter('const_curr', 0.0)
        self.declare_parameter('const_brake', 0.0)
        self.declare_parameter('accel_time', 1.0)
        self.declare_parameter('decel_time', 1.0)
        self.declare_parameter('const_accel', 0.0)
        self.declare_parameter('const_decel', 0.0)
        self.declare_parameter('angle_time', 5.0)
        self.declare_parameter('start_pos', 0.5)
        self.declare_parameter('end_pos', 0.5)
        self.declare_parameter('start_angle', 0.0)
        self.declare_parameter('end_angle', 0.0)
        self.declare_parameter('const_speed', 1.0)
        self.declare_parameter('acc_profile', 1)
        self.declare_parameter('period', 1.0)
        self.declare_parameter('repetitions', 5)
        self.declare_parameter('bangbang_steer', 0.0)

        # Retrieve parameter values
        self.experiment = self.get_parameter('experiment').value
        self.const_erpm = self.get_parameter('const_erpm').value
        self.const_servo = self.get_parameter('const_servo').value
        self.const_curr = self.get_parameter('const_curr').value
        self.const_brake = self.get_parameter('const_brake').value
        self.accel_time = self.get_parameter('accel_time').value
        self.decel_time = self.get_parameter('decel_time').value
        self.const_accel = self.get_parameter('const_accel').value
        self.const_decel = self.get_parameter('const_decel').value
        self.angle_time = self.get_parameter('angle_time').value
        self.start_pos = self.get_parameter('start_pos').value
        self.end_pos = self.get_parameter('end_pos').value
        self.start_angle = self.get_parameter('start_angle').value
        self.end_angle = self.get_parameter('end_angle').value
        self.const_speed = self.get_parameter('const_speed').value
        self.acc_profile = self.get_parameter('acc_profile').value
        self.period = self.get_parameter('period').value
        self.repetitions = self.get_parameter('repetitions').value
        self.bangbang_steer = self.get_parameter('bangbang_steer').value

        self.get_logger().info(f"Starting experiment #{self.experiment}")

        # Keep track of logged states to avoid output flooding
        self.last_log_state = None

        self.start_time = self.get_clock().now()
        
        # Timer running at 30Hz (0.0333 seconds)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def log_state(self, state_str):
        if self.last_log_state != state_str:
            self.get_logger().info(state_str)
            self.last_log_state = state_str

    def timer_callback(self):
        if self.experiment == 1:
            self.send_const_vesc_cmd()
        elif self.experiment == 2:
            self.accel_decel()
        elif self.experiment == 3:
            self.drive_accel_decel()
        elif self.experiment == 4:
            self.increase_servo_position()
        elif self.experiment == 5:
            self.increase_steering_angle()
        elif self.experiment == 6:
            self.acc_profile_recorder()
        elif self.experiment == 7:
            self.bang_bang_servo()

    def send_const_vesc_cmd(self):
        erpm_msg = Float64()
        erpm_msg.data = float(self.const_erpm)
        servo_msg = Float64()
        servo_msg.data = float(self.const_servo)
        self.erpm_pub.publish(erpm_msg)
        self.servo_pub.publish(servo_msg)

    def accel_decel(self):
        servo_msg = Float64()
        servo_msg.data = float(self.const_servo)
        self.servo_pub.publish(servo_msg)
        
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.accel_time:
            current_msg = Float64()
            current_msg.data = float(self.const_curr)
            self.current_pub.publish(current_msg)
            self.log_state("accelerating")
        elif elapsed < (self.accel_time + self.decel_time):
            brake_msg = Float64()
            brake_msg.data = float(self.const_brake)
            self.brake_pub.publish(brake_msg)
            self.log_state("decelerating")
        else:
            self.log_state("experiment over")

    def drive_accel_decel(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = 0.0
        drive_msg.drive.steering_angle_velocity = 0.0
        drive_msg.drive.speed = 0.0
        drive_msg.drive.jerk = 512.0  # flag to indicate using acceleration instead of speed
        
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < 3.0:
            self.log_state("experiment starting")
        elif elapsed < (self.accel_time + 3.0):
            drive_msg.drive.acceleration = float(self.const_accel)
            self.log_state("accelerating")
            self.cmd_pub.publish(drive_msg)
        elif elapsed < (self.accel_time + self.decel_time + 3.0):
            drive_msg.drive.acceleration = float(self.const_decel)
            self.log_state("decelerating")
            self.cmd_pub.publish(drive_msg)
        else:
            self.log_state("experiment over")

    def increase_servo_position(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        time_frac = elapsed / self.angle_time
        if time_frac <= 1.0:
            angle = self.end_pos * time_frac + self.start_pos * (1.0 - time_frac)
            servo_msg = Float64()
            servo_msg.data = float(angle)
            erpm_msg = Float64()
            erpm_msg.data = float(self.const_erpm)
            self.erpm_pub.publish(erpm_msg)
            self.servo_pub.publish(servo_msg)
        else:
            self.log_state("experiment over")

    def increase_steering_angle(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        time_frac = elapsed / self.angle_time
        if time_frac <= 1.0:
            angle = self.end_angle * time_frac + self.start_angle * (1.0 - time_frac)
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = self.get_clock().now().to_msg()
            drive_msg.drive.steering_angle = float(angle)
            drive_msg.drive.steering_angle_velocity = 0.0
            drive_msg.drive.speed = float(self.const_speed)
            drive_msg.drive.acceleration = 0.0
            drive_msg.drive.jerk = 0.0
            self.cmd_pub.publish(drive_msg)
            if time_frac >= 0.9:
                self.log_state("Ending soon, stop the bag")
        else:
            self.log_state("experiment over")

    def acc_profile_recorder(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = 0.0
        drive_msg.drive.steering_angle_velocity = 0.0
        
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if self.acc_profile == 1:  # Step response to speed
            speed = 3.0
            duration = 4.0
            if elapsed < 2.0:
                drive_msg.drive.speed = 0.0
                drive_msg.drive.jerk = 0.0
                drive_msg.drive.acceleration = 0.0
                self.cmd_pub.publish(drive_msg)
                self.log_state("experiment starting")
            elif elapsed > (duration + 2.0):
                drive_msg.drive.speed = 0.0
                drive_msg.drive.jerk = 0.0
                drive_msg.drive.acceleration = 0.0
                self.cmd_pub.publish(drive_msg)
                self.log_state("experiment over")
            else:
                drive_msg.drive.speed = speed
                drive_msg.drive.jerk = 0.0
                drive_msg.drive.acceleration = 0.0
                self.cmd_pub.publish(drive_msg)
                self.log_state("accelerating")
                
        elif self.acc_profile == 2:  # Max acc and braking test
            drive_msg.drive.speed = 0.0
            drive_msg.drive.jerk = 512.0
            if elapsed < self.accel_time:
                drive_msg.drive.acceleration = 100.0
                self.log_state("accelerating")
                self.cmd_pub.publish(drive_msg)
            elif elapsed < (self.accel_time + self.decel_time):
                drive_msg.drive.acceleration = -100.0
                self.cmd_pub.publish(drive_msg)
                self.log_state("decelerating")
            else:
                self.log_state("experiment over")
                
        elif self.acc_profile == 3:  # speed steps at speed
            speed1 = 3.0
            speed2 = 5.0
            duration = 6.0
            if elapsed < 2.0 or elapsed > (2.0 + duration):
                drive_msg.drive.speed = 0.0
                drive_msg.drive.jerk = 0.0
                drive_msg.drive.acceleration = 0.0
                self.cmd_pub.publish(drive_msg)
                self.log_state("standing still")
            elif elapsed > (duration / 3.0 + 2.0) and elapsed < (duration / 3.0 * 2.0 + 2.0):
                drive_msg.drive.speed = speed2
                drive_msg.drive.jerk = 0.0
                drive_msg.drive.acceleration = 0.0
                self.cmd_pub.publish(drive_msg)
                self.log_state("accelerating further")
            elif elapsed > (duration / 3.0 * 2.0 + 2.0) and elapsed < (duration + 2.0):
                drive_msg.drive.speed = speed2
                drive_msg.drive.jerk = 0.0
                drive_msg.drive.acceleration = 0.0
                self.cmd_pub.publish(drive_msg)
                self.log_state("decelerating")
            else:
                drive_msg.drive.speed = speed1
                drive_msg.drive.jerk = 0.0
                drive_msg.drive.acceleration = 0.0
                self.cmd_pub.publish(drive_msg)
                self.log_state("accelerating")
                
        else:
            self.log_state("invalid speed profile")
            drive_msg.drive.speed = 1.0
            drive_msg.drive.jerk = 512.0
            if elapsed < self.accel_time:
                drive_msg.drive.acceleration = float(self.const_accel)
                self.cmd_pub.publish(drive_msg)
            elif elapsed < (self.accel_time + self.decel_time):
                drive_msg.drive.acceleration = float(self.const_decel)
                self.cmd_pub.publish(drive_msg)
            else:
                self.log_state("experiment over")

    def bang_bang_servo(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        time_frac = elapsed / self.period
        
        steer_sign = -1.0
        if (time_frac % 1.0) < 0.5:
            steer_sign = 1.0
            
        if time_frac <= float(self.repetitions):
            angle = steer_sign * self.bangbang_steer
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = self.get_clock().now().to_msg()
            drive_msg.drive.steering_angle = float(angle)
            drive_msg.drive.steering_angle_velocity = 0.0
            drive_msg.drive.speed = float(self.const_speed)
            drive_msg.drive.acceleration = 0.0
            drive_msg.drive.jerk = 0.0
            self.cmd_pub.publish(drive_msg)
        else:
            self.log_state("experiment over")

def main(args=None):
    rclpy.init(args=args)
    node = IDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
