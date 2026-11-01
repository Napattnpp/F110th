#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import time
import math
from f110_msgs.msg import ObstacleArray
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class ObstacleTelemetryRecorder(Node):
    def __init__(self):
        super().__init__('obstacle_telemetry_recorder')
        
        self.output_path = "/home/jetson/F110th/out/obstacle_telemetry_live.jsonl"
        self.log_file = open(self.output_path, "a")
        
        # State caches
        self.cur_state = "UNKNOWN"
        self.cur_frenet = None
        self.cur_cartesian = None
        self.cur_drive = None
        self.track_length = 18.82 # default for yee1/yee2, updated if known
        
        # Subscriptions
        self.sub_state = self.create_subscription(String, '/state', self.state_cb, 10)
        self.sub_frenet = self.create_subscription(Odometry, '/car_state/frenet/odom', self.frenet_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, '/car_state/odom', self.cartesian_cb, 10)
        self.sub_drive = self.create_subscription(AckermannDriveStamped, '/drive', self.drive_cb, 10)
        self.sub_obstacles = self.create_subscription(ObstacleArray, '/perception/obstacles', self.obstacle_cb, 10)
        
        # Periodic snapshot timer at 20 Hz
        self.timer = self.create_timer(0.05, self.timer_cb)
        self.last_obs_cache = []
        self.last_alert_time = 0.0
        
        self.get_logger().info(f"Obstacle Telemetry Recorder started -> Logging to {self.output_path}")

    def state_cb(self, msg):
        self.cur_state = msg.data

    def frenet_cb(self, msg):
        self.cur_frenet = {
            's': msg.pose.pose.position.x,
            'd': msg.pose.pose.position.y,
            'vs': msg.twist.twist.linear.x,
            'vd': msg.twist.twist.linear.y
        }

    def cartesian_cb(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
        self.cur_cartesian = {
            'x': p.x,
            'y': p.y,
            'yaw': yaw,
            'vx': msg.twist.twist.linear.x
        }

    def drive_cb(self, msg):
        self.cur_drive = {
            'speed': msg.drive.speed,
            'steering': msg.drive.steering_angle
        }

    def obstacle_cb(self, msg):
        obs_list = []
        for obs in msg.obstacles:
            obs_list.append({
                'id': obs.id,
                's_center': obs.s_center,
                'd_center': obs.d_center,
                'size': obs.size,
                'vs': obs.vs,
                'vd': obs.vd,
                'is_static': obs.is_static,
                'is_visible': obs.is_visible
            })
        self.last_obs_cache = obs_list

    def timer_cb(self):
        t_now = time.time()
        record = {
            'timestamp': t_now,
            'state': self.cur_state,
            'frenet': self.cur_frenet,
            'cartesian': self.cur_cartesian,
            'drive': self.cur_drive,
            'obstacles': self.last_obs_cache
        }
        
        self.log_file.write(json.dumps(record) + "\n")
        self.log_file.flush()
        
        # Live Alert when obstacle is active
        if self.last_obs_cache and self.cur_frenet:
            cur_s = self.cur_frenet['s']
            cur_vs = self.cur_frenet['vs']
            for obs in self.last_obs_cache:
                gap = (obs['s_center'] - cur_s) % self.track_length
                if gap < 6.0 and (t_now - self.last_alert_time) > 0.5:
                    self.last_alert_time = t_now
                    d_cmd = self.cur_drive['speed'] if self.cur_drive else 0.0
                    print(f"[{time.strftime('%H:%M:%S')}] [OBS DETECTED] id={obs['id']} gap={gap:.2f}m | car_s={cur_s:.2f}m vs={cur_vs:.2f}m/s | state={self.cur_state} cmd_spd={d_cmd:.2f}m/s", flush=True)

def main():
    rclpy.init()
    node = ObstacleTelemetryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.log_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

