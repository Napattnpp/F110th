#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from f110_msgs.msg import WpntArray
from visualization_msgs.msg import Marker, MarkerArray

# the `.` tells Python to look in the current directory at runtime
from .readwrite_global_waypoints import read_global_waypoints

class GlobalRepublisher(Node):
    """
    Node for publishing the global waypoints/markers and track bounds markers frequently after they have been calculated
    """
    def __init__(self):
        super().__init__('global_republisher_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.glb_markers = None
        self.glb_wpnts = None
        self.track_bounds = None
        self.publish_markers = self.get_parameter('publish_markers').get_parameter_value().bool_value
        

        # Initialize all attributes to None to avoid AttributeError during callbacks/loops
        self.glb_wpnts = None
        self.glb_markers = None
        self.glb_sp_wpnts = None
        self.glb_sp_markers = None
        self.centerline_wpnts = None
        self.centerline_markers = None
        self.track_bounds = None
        self.map_infos = None
        self.est_lap_time = None

        #Declare local ros params
        if not self.has_parameter('track_length'):
            self.declare_parameter(name='track_length', value=6.9)
        if not self.has_parameter('trajectory_type'):
            self.declare_parameter(name='trajectory_type', value='minimum_curvature')

        self.glb_wpnts_pub = self.create_publisher(WpntArray, '/global_waypoints', 10)
        if self.publish_markers:
            self.glb_markers_pub = self.create_publisher(MarkerArray, '/global_waypoints/markers', 10)
            self.vis_track_bnds = self.create_publisher(MarkerArray, '/trackbounds/markers', 10)

        # shortest_path
        self.publish_shortest_path = self.get_parameter('publish_shortest_path').get_parameter_value().bool_value
        if self.publish_shortest_path:
            self.get_logger().info("Global Republisher: Publishing shortest path waypoints and markers")
            self.glb_sp_wpnts_pub = self.create_publisher(WpntArray, '/global_waypoints/shortest_path', 10)
            if self.publish_markers:
                self.glb_sp_markers_pub = self.create_publisher(MarkerArray, '/global_waypoints/shortest_path/markers', 10)

        # centerline
        self.publish_centerline = self.get_parameter('publish_centerline').get_parameter_value().bool_value
        if self.publish_centerline:
            self.get_logger().info("Global Republisher: Publishing centerline waypoints and markers")
            self.centerline_wpnts = None
            self.centerline_markers = None
            self.centerline_wpnts_pub = self.create_publisher(WpntArray, '/centerline_waypoints', 10)
            if self.publish_markers:
                self.centerline_markers_pub = self.create_publisher(MarkerArray, '/centerline_waypoints/markers', 10)

        # map infos
        self.map_infos = None
        self.map_info_pub = self.create_publisher(String, 'map_infos', 10)

        self.est_lap_time = None
        self.est_lap_time_pub = self.create_publisher(Float32, 'estimated_lap_time', 10)

        # graph lattice
        self.publish_lattice = self.get_parameter('publish_lattice').get_parameter_value().bool_value
        if self.publish_lattice:
            self.get_logger().info("Global Republisher: Publishing graph lattice")
            self.graph_lattice = None
            self.lattice_pub = self.create_publisher(MarkerArray, '/lattice_viz', 10)

        # Read info from json file if it is provided, so everything is always published
        map_path = self.get_parameter('map_path').get_parameter_value().string_value
        if map_path:
            self.get_logger().info(f"Reading parameters from {map_path}")
            try:
                (
                    self.map_infos, self.est_lap_time, self.centerline_markers,
                    self.centerline_wpnts, self.glb_markers, self.glb_wpnts,
                    self.glb_sp_markers, self.glb_sp_wpnts, self.track_bounds
                ) = read_global_waypoints(map_dir=map_path)
            except FileNotFoundError:
                self.get_logger().warn(f"{map_path} param not found. Not publishing")
        else:
            self.get_logger().info(f"global_trajectory_publisher did not find any map_path param {map_path}")
        
        # Publish immediately and then at 1 Hz
        self.global_republisher()
        self.create_timer(3.0, self.global_republisher)
        
    def glb_wpnts_cb(self, msg):
        self.glb_wpnts = msg
        track_length = msg.wpnts[-1].s_m
        gb_len = rclpy.Parameter('track_length', rclpy.Parameter.Type.DOUBLE, track_length)
        self.set_parameters([gb_len])

    def glb_markers_cb(self, msg):
        self.glb_markers = msg

    def glb_sp_wpnts_cb(self, msg):
        self.glb_sp_wpnts = msg

    def glb_sp_markers_cb(self, msg):
        self.glb_sp_markers = msg

    def centerline_wpnt_cb(self, msg):
        self.centerline_wpnts = msg

    def centerline_markers_cb(self, msg):
        self.centerline_markers = msg

    def bounds_cb(self, msg):
        self.track_bounds = msg

    def map_info_cb(self, msg):
        self.map_infos = msg

    def est_lap_time_cb(self, msg):
        self.est_lap_time = msg

    def lattice_cb(self, msg):
        self.graph_lattice = msg

    def global_republisher(self):
        traj_type = self.get_parameter('trajectory_type').get_parameter_value().string_value

        # Select which trajectory to publish on /global_waypoints
        if traj_type == 'shortest_path':
            wpnts_to_pub = self.glb_sp_wpnts
            markers_to_pub = self.glb_sp_markers
        elif traj_type == 'centerline':
            wpnts_to_pub = self.centerline_wpnts
            markers_to_pub = self.centerline_markers
            if wpnts_to_pub is not None:
                for wp in wpnts_to_pub.wpnts:
                    if wp.vx_mps == 0.0:
                        wp.vx_mps = 2.0
        else: # default to minimum_curvature
            wpnts_to_pub = self.glb_wpnts
            markers_to_pub = self.glb_markers

        # Defensive reconstruction for Foxglove: ensure markers_to_pub has non-zero scale and real coordinates
        needs_rebuild = False
        if markers_to_pub is None or not hasattr(markers_to_pub, 'markers') or len(markers_to_pub.markers) == 0:
            needs_rebuild = True
        elif len(markers_to_pub.markers) > 0:
            m0 = markers_to_pub.markers[0]
            if getattr(getattr(m0, 'scale', None), 'x', 0.0) == 0.0:
                needs_rebuild = True

        if needs_rebuild and wpnts_to_pub is not None and hasattr(wpnts_to_pub, 'wpnts') and len(wpnts_to_pub.wpnts) > 0:
            rebuilt = MarkerArray()
            max_vx = max([w.vx_mps for w in wpnts_to_pub.wpnts]) if len(wpnts_to_pub.wpnts) > 0 else 1.0
            max_vx = max(max_vx, 1.0)
            color_rgb = (1.0, 0.85, 0.0) if traj_type == 'shortest_path' else (0.0, 0.85, 1.0)
            for i, w in enumerate(wpnts_to_pub.wpnts):
                m = Marker()
                m.header.frame_id = 'map'
                m.id = i
                m.type = Marker.CYLINDER
                m.action = Marker.ADD
                m.scale.x = 0.12
                m.scale.y = 0.12
                m.scale.z = max(0.15, float(w.vx_mps / max_vx * 0.5))
                m.color.r = float(color_rgb[0])
                m.color.g = float(color_rgb[1])
                m.color.b = float(color_rgb[2])
                m.color.a = 1.0
                m.pose.position.x = float(w.x_m)
                m.pose.position.y = float(w.y_m)
                m.pose.position.z = m.scale.z / 2.0
                m.pose.orientation.w = 1.0
                rebuilt.markers.append(m)
            markers_to_pub = rebuilt
            if traj_type == 'shortest_path':
                self.glb_sp_markers = rebuilt
            elif traj_type == 'centerline':
                self.centerline_markers = rebuilt
            else:
                self.glb_markers = rebuilt

        # Ensure all markers have frame_id set to 'map' and bright visible colors
        for ma in [markers_to_pub, self.glb_sp_markers, self.centerline_markers, self.track_bounds]:
            if ma is not None and hasattr(ma, 'markers'):
                for m in ma.markers:
                    if not m.header.frame_id:
                        m.header.frame_id = 'map'
                    # Force bright non-black colors so markers pop out on dark maps
                    if m.color.r == 0.0 and m.color.g == 0.0 and m.color.b == 0.0:
                        m.color.r = 0.0
                        m.color.g = 0.85
                        m.color.b = 1.0
                        m.color.a = 1.0

        # Publish selected global waypoints
        if wpnts_to_pub is not None:
            self.glb_wpnts_pub.publish(wpnts_to_pub)
        if self.publish_markers and markers_to_pub is not None and hasattr(self, 'glb_markers_pub'):
            self.glb_markers_pub.publish(markers_to_pub)
        if self.publish_markers and self.track_bounds is not None and hasattr(self, 'vis_track_bnds'):
            self.vis_track_bnds.publish(self.track_bounds)
        
        # Publish shortest path
        if self.publish_shortest_path:
            if self.glb_sp_wpnts is not None and hasattr(self, 'glb_sp_wpnts_pub'):
                self.glb_sp_wpnts_pub.publish(self.glb_sp_wpnts)
            if self.publish_markers and self.glb_sp_markers is not None and hasattr(self, 'glb_sp_markers_pub'):
                self.glb_sp_markers_pub.publish(self.glb_sp_markers)
        
        # Publish centerline
        if self.publish_centerline:
            if self.centerline_wpnts is not None and hasattr(self, 'centerline_wpnts_pub'):
                self.centerline_wpnts_pub.publish(self.centerline_wpnts)
            if self.publish_markers and self.centerline_markers is not None and hasattr(self, 'centerline_markers_pub'):
                self.centerline_markers_pub.publish(self.centerline_markers)
        
        # Publish track bounds
        if self.publish_markers and self.track_bounds is not None and hasattr(self, 'vis_track_bnds'):
            self.vis_track_bnds.publish(self.track_bounds)
        
        # Publish map info and lap time (always)
        if self.map_infos is not None:
            self.map_info_pub.publish(self.map_infos)
        if self.est_lap_time is not None:
            self.est_lap_time_pub.publish(self.est_lap_time)
        
        # Publish lattice
        if self.publish_lattice and self.graph_lattice is not None and hasattr(self, 'lattice_pub'):
            self.lattice_pub.publish(self.graph_lattice)

         

def main(args=None):
    rclpy.init(args=args)
    republisher = GlobalRepublisher()
    rclpy.spin(republisher)
    republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
