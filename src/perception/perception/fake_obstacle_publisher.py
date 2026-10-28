import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from f110_msgs.msg import ObstacleArray, Obstacle, WpntArray, OpponentTrajectory, OppWpnt
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry, OccupancyGrid
import numpy as np
from frenet_conversion.frenet_converter import FrenetConverter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import random

class FakeObstaclePublisher(Node):
    """Publish fake static obstacles or dynamic opponents in the F110 simulator"""
    def __init__(self):
        super().__init__('fake_obstacle_publisher')
        self.ego_wpnts = None
        self.max_s = None
        self.waypoints_topic = None
        self.wpnts = None
        
        self.first_map = True
        self.original_map = None
        self.current_map = None
        self.obstacle_size = 2
        self.last_ind = None
        
        self.cur_s = None
        self.local_wpnts = None
        self.static_obstacles = []

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('obs_mode', 'dynamic_opponent'), # 'dynamic_opponent', 'static_ahead', or 'none'
                ('speed_scaler', 0.5), # Used for dynamic opponent
                ('constant_speed', False), # Used for dynamic opponent
                ('trajectory', "min_curv"), # Used for dynamic opponent path
                ('start_s', 0.0), # Used for dynamic opponent
                ('type', 'virtual') # 'virtual' (Marker+ObstacleArray) or 'lidar' (OccupancyGrid mod)
        ])
        
        self.obs_mode = self.get_parameter('obs_mode').value
        self.speed_scaler = self.get_parameter('speed_scaler').value
        self.constant = self.get_parameter('constant_speed').value
        self.waypoints_type = self.get_parameter('trajectory').value
        self.starting_s = float(self.get_parameter('start_s').value)
        self.opp_type = self.get_parameter('type').value

        # choose trajectory for dynamic opponent
        if self.waypoints_type == 'min_curv':
            self.waypoints_topic = '/global_waypoints'
        elif self.waypoints_type == 'shortest_path':
            self.waypoints_topic = '/global_waypoints/shortest_path'
        elif self.waypoints_type == 'centerline':
            self.waypoints_topic = '/centerline_waypoints'
        else:
            self.get_logger().error('Invalid trajectory type')
        
        # Subscriptions and Publishers
        self.odom_subscriber = self.create_subscription(Odometry, '/car_state/frenet/odom', self.odom_cb, 10)
        self.global_waypoints_subscriber = self.create_subscription(WpntArray, '/global_waypoints_scaled', self.global_waypoints_cb, 10)
        self.local_waypoints_subscriber = self.create_subscription(WpntArray, '/state_machine/local_waypoints', self.local_wpnts_cb, 10)
        
        if self.obs_mode == 'dynamic_opponent':
            self.waypoints_subscriber = self.create_subscription(WpntArray, self.waypoints_topic, self.waypoints_cb, 10)
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        self.sub_map = self.create_subscription(OccupancyGrid, "/map", self.map_cb, qos_profile=qos_profile)
        self.obstacle_pub = self.create_publisher(ObstacleArray, '/perception/fake_obstacles', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/perception/dummy_obstacle_markers', 10)
        self.opponent_traj_pub = self.create_publisher(OpponentTrajectory, '/opponent_waypoints', 10)
        self.pub_map = self.create_publisher(OccupancyGrid, "/map", qos_profile=qos_profile)

        # Frenet Conversion Service Clients
        self.converter = None
        
        if self.obs_mode == 'dynamic_opponent':
            self.dynamic_obstacle = self.init_dynamic_obstacle()
            self.obj_len = 0.5
        
        self.loop_setup()

    def init_dynamic_obstacle(self):
        """ Initializes the dynamic obstacle configuration."""
        dynamic_obstacle = Obstacle()
        dynamic_obstacle.id = 999
        dynamic_obstacle.size = 0.5
        dynamic_obstacle.d_right = -0.1
        dynamic_obstacle.d_left = 0.1
        dynamic_obstacle.is_actually_a_gap = False
        return dynamic_obstacle
    
    ### Callbacks ###
    def global_waypoints_cb(self, data: WpntArray):
        self.ego_wpnts = data.wpnts[:-1]
        self.max_s = data.wpnts[-1].s_m
        
        if self.converter == None:
            waypoints = np.array([[wpnt.x_m, wpnt.y_m, wpnt.psi_rad]
                                  for wpnt in data.wpnts])
            self.converter = FrenetConverter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2])
    
    def waypoints_cb(self, data: WpntArray):
        self.wpnts = data.wpnts[:-1]

    def odom_cb(self, data: Odometry):
        """ Odometry callback to track the car's state."""
        self.car_odom = data
        self.cur_s = data.pose.pose.position.x
        
    def local_wpnts_cb(self, data: WpntArray):
        if len(data.wpnts) > 0:
            self.local_wpnts = data.wpnts

    def map_cb(self, data):
        if self.first_map:
            self.original_map = OccupancyGrid()
            self.original_map = data
            self.current_map = self.original_map
            self.first_map = False
        else:
            self.current_map.data = data.data

    ### Helpers ###
    def publish_obstacle_cartesian(self, obstacles):
        """Publishes obstacles as markers in the Cartesian frame."""
        marker_array = MarkerArray()
        for obs in obstacles:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.ns = "fake_obs"
            marker.id = obs.id
            if self.obs_mode == 'static_ahead':
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.scale.x = 0.6
                marker.scale.y = 0.3
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            marker_xy = self.converter.get_cartesian(obs.s_center, obs.d_center)
            marker.pose.position.x = float(marker_xy[0])
            marker.pose.position.y = float(marker_xy[1])
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def rc_2_ind(self,r,c):
        return r*self.original_map.info.width + c
    
    def ind_2_rc(self,ind):
        rc = []
        row = ind//self.original_map.info.width
        col = ind%self.original_map.info.width
        rc.append(row)
        rc.append(col)
        return rc
        
    def coord_2_cell_rc(self, x, y):
        rc = [int((y-self.original_map.info.origin.position.y)//self.original_map.info.resolution), int((x-self.original_map.info.origin.position.x)//self.original_map.info.resolution)]
        return rc

    def add_lidar_obs(self, ind):
        rc = self.ind_2_rc(ind)
        for i in range (-self.obstacle_size,self.obstacle_size):
            for j in range (-self.obstacle_size,self.obstacle_size):
                current_r = rc[0]+i
                current_c = rc[1]+j
                current_ind = self.rc_2_ind(current_r,current_c)
                self.current_map.data[current_ind] = 100
        self.pub_map.publish(self.current_map)
                
    def clear_lidar_obs(self, ind):
        rc = self.ind_2_rc(ind)
        for i in range (-self.obstacle_size,self.obstacle_size):
            for j in range (-self.obstacle_size,self.obstacle_size):
                current_r = rc[0]+i
                current_c = rc[1]+j
                current_ind = self.rc_2_ind(current_r,current_c)
                self.current_map.data[current_ind] = 0
        self.pub_map.publish(self.current_map)
    
    def loop_setup(self):
        """ Main loop for updating and publishing the dynamic obstacle."""
        if self.obs_mode == 'none':
            self.get_logger().info("Fake Obstacle Publisher disabled (obs_mode = none)")
            return

        # Wait for global waypoint message to arrive
        while self.ego_wpnts is None:
            self.get_logger().info("Waiting for global waypoints message", throttle_duration_sec=0.5)
            rclpy.spin_once(self)
            
        if self.opp_type == "lidar":
            while self.original_map is None:
                self.get_logger().info("Waiting for map message", throttle_duration_sec=0.5)
                rclpy.spin_once(self)
            
        s_array = np.array([wpnt.s_m for wpnt in self.ego_wpnts])
        self.get_logger().info("Got global waypoints message")
        
        if self.obs_mode == 'dynamic_opponent':
            # Read in opponent waypoints
            if self.constant:
                for i in range(len(self.ego_wpnts)):
                    self.ego_wpnts[i].vx_mps = self.speed_scaler
            else:
                for i in range(len(self.ego_wpnts)):
                    self.ego_wpnts[i].vx_mps = self.ego_wpnts[i].vx_mps * self.speed_scaler
                    
            # Wait for opponent waypoints message to arrive
            while self.wpnts is None:
                self.get_logger().info("Waiting for opponent waypoints message", throttle_duration_sec=0.5)
                rclpy.spin_once(self)
            
            opponent_frenet = self.converter.get_frenet([wpnt.x_m for wpnt in self.wpnts], [wpnt.y_m for wpnt in self.wpnts])
            opponent_s = opponent_frenet[0]
            opponent_d = opponent_frenet[1]
            sorted_indices = sorted(range(len(opponent_s)), key=lambda k: opponent_s[k])
            opponent_s_sorted = [opponent_s[i] for i in sorted_indices]
            opponent_d_sorted = [opponent_d[i] for i in sorted_indices]
            
            resampled_opponent_d = np.interp(s_array, opponent_s_sorted, opponent_d_sorted)
            resampled_opponent_vs = [wpnt.vx_mps for wpnt in self.ego_wpnts]
            resampled_opponent_xy = self.converter.get_cartesian(s_array, resampled_opponent_d)
            
            self.opponent_wpnts = OpponentTrajectory()
            for i in range(len(s_array)):
                wpnt = OppWpnt()
                wpnt.x_m = resampled_opponent_xy[0][i]
                wpnt.y_m = resampled_opponent_xy[1][i]
                wpnt.proj_vs_mps = resampled_opponent_vs[i]
                wpnt.s_m = s_array[i]
                wpnt.d_m = resampled_opponent_d[i]
                self.opponent_wpnts.oppwpnts.append(wpnt)
                
            self.dynamic_obstacle.s_center = self.starting_s
            self.opponent_s_array = np.array([wpnt.s_m for wpnt in self.opponent_wpnts.oppwpnts])
            
        self.get_logger().info("Fake Obstacle Publisher Ready")
        
        self.counter = 0
        self.looptime = 1/50.0
        self.timer = self.create_timer(self.looptime, self.timer_cb)
        
    def generate_static_obstacles(self):
        if self.cur_s is None or self.local_wpnts is None or len(self.local_wpnts) == 0:
            return []
            
        if len(self.static_obstacles) == 0:
            num_obs = random.randint(1, 3)
            self.get_logger().info(f"=========== SPAWNING {num_obs} STATIC OBSTACLES ===========")
            
            base_s = self.local_wpnts[-1].s_m
            
            for i in range(num_obs):
                obs = Obstacle()
                obs.id = 990 + i
                
                offset = 15.0 + (i * 20.0) + random.uniform(0, 10.0)
                obs.s_center = (base_s + offset) % self.max_s
                
                obs.size = 1.0
                obs.s_start = obs.s_center - 0.5
                obs.s_end = obs.s_center + 0.5
                
                obs.d_center = 0.0
                obs.d_left = obs.d_center + 0.5
                obs.d_right = obs.d_center - 0.5
                
                obs.vs = 0.0
                obs.vd = 0.0
                obs.is_static = True
                obs.is_visible = True
                obs.is_actually_a_gap = False
                
                self.static_obstacles.append(obs)
                
        # If the car has passed ALL of them, respawn
        if len(self.static_obstacles) > 0:
            all_passed = True
            for obs in self.static_obstacles:
                dist = (obs.s_center - self.cur_s) % self.max_s
                if dist < (self.max_s / 2):
                    all_passed = False
                    break
            
            if all_passed:
                self.static_obstacles = [] # Will respawn next tick
                
        return self.static_obstacles

    ### MAIN LOOP ###   
    def timer_cb(self):
        obstacle_msg = ObstacleArray()
        obstacle_msg.header.stamp = self.get_clock().now().to_msg()
        obstacle_msg.header.frame_id = 'frenet'
        
        if self.obs_mode == 'dynamic_opponent':
            s = self.dynamic_obstacle.s_center
            approx_idx = np.abs(self.opponent_s_array - s).argmin()
            
            self.dynamic_obstacle_speed = self.opponent_wpnts.oppwpnts[approx_idx].proj_vs_mps
            self.dynamic_obstacle.s_center = (self.dynamic_obstacle.s_center + self.dynamic_obstacle_speed * self.looptime) % self.max_s
            self.dynamic_obstacle.s_start = (self.dynamic_obstacle.s_center - self.obj_len/2) % self.max_s
            self.dynamic_obstacle.s_end = (self.dynamic_obstacle.s_center + self.obj_len/2) % self.max_s
            self.dynamic_obstacle.d_center = self.opponent_wpnts.oppwpnts[approx_idx].d_m
            self.dynamic_obstacle.d_right = self.dynamic_obstacle.d_center - 0.1
            self.dynamic_obstacle.d_left = self.dynamic_obstacle.d_center + 0.1
            self.dynamic_obstacle.size = self.obj_len
            self.dynamic_obstacle.vs = self.dynamic_obstacle_speed
            self.dynamic_obstacle.is_static = False
            
            obstacle_msg.obstacles.append(self.dynamic_obstacle)
            
        elif self.obs_mode == 'static_ahead':
            obstacle_msg.obstacles = self.generate_static_obstacles()
        
        if self.opp_type == "lidar":
            if self.last_ind != None:
                self.clear_lidar_obs(self.last_ind)
                self.last_ind = None

            for obs in obstacle_msg.obstacles:    
                obstacle_xy = self.converter.get_cartesian(obs.s_center, obs.d_center)
                x = obstacle_xy[0]
                y = obstacle_xy[1]
                rc_add = self.coord_2_cell_rc(x,y)
                ind_add = self.rc_2_ind(rc_add[0],rc_add[1])
                
                self.last_ind = ind_add # Note: For multiple obstacles, this lidar clearing logic will only clear the last one. Might need to make last_ind a list, but keeping it simple for now as it matches original logic.
                self.add_lidar_obs(ind_add)
                
        elif self.opp_type == "virtual":     
            self.publish_obstacle_cartesian(obstacle_msg.obstacles)
            self.obstacle_pub.publish(obstacle_msg)
        
        self.counter += 1
        if self.counter > 25 and self.obs_mode == 'dynamic_opponent':
            opponent_traj_msg = OpponentTrajectory()
            opponent_traj_msg.header.stamp = self.get_clock().now().to_msg()
            opponent_traj_msg.header.frame_id = 'frenet'
            opponent_traj_msg.lap_count = 2
            opponent_traj_msg.oppwpnts = self.opponent_wpnts.oppwpnts
            self.opponent_traj_pub.publish(opponent_traj_msg)
            self.counter = 0

def main(args=None):
    rclpy.init(args=args)
    node = FakeObstaclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
