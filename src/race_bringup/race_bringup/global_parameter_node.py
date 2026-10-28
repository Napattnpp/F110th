import rclpy
from rclpy.node import Node


class GlobalParameterNode(Node):
    def __init__(self):
        super().__init__('global_parameters')
        self.declare_parameter('racecar_version', 'ORIN1')
        self.declare_parameter('sim', False)
        self.declare_parameter('map_path', '')
        self.get_logger().info('Global parameters are available.')


def main(args=None):
    rclpy.init(args=args)
    node = GlobalParameterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
