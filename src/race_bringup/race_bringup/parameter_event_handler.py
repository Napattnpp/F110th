from rcl_interfaces.msg import ParameterEvent
from rclpy.parameter import Parameter

class ParameterEventHandler:
    def __init__(self, node):
        self.node = node
        self.event_callbacks = []
        self.parameter_callbacks = {}  # key: (node_name, parameter_name), value: list of callbacks
        
        # Subscribe to /parameter_events
        self.sub = self.node.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self._event_callback,
            10
        )

    def add_parameter_event_callback(self, callback):
        self.event_callbacks.append(callback)
        return callback

    def add_parameter_callback(self, parameter_name, callback, node_name=None):
        if node_name is None:
            node_name = self.node.get_fully_qualified_name()
        
        key = (node_name, parameter_name)
        if key not in self.parameter_callbacks:
            self.parameter_callbacks[key] = []
        self.parameter_callbacks[key].append(callback)
        return callback

    def _event_callback(self, event_msg):
        # 1. Call event callbacks with the event message
        for callback in self.event_callbacks:
            try:
                callback(event_msg)
            except Exception as e:
                self.node.get_logger().error(f"Error in parameter event callback: {e}")

        # 2. Extract node name
        event_node = event_msg.node.lstrip('/')
        
        # 3. Process new and changed parameters
        for p_msg in event_msg.new_parameters + event_msg.changed_parameters:
            param = Parameter.from_parameter_msg(p_msg)
            
            for (node_name, parameter_name), callbacks in self.parameter_callbacks.items():
                norm_node_name = node_name.lstrip('/')
                if event_node == norm_node_name and param.name == parameter_name:
                    for callback in callbacks:
                        try:
                            callback(param)
                        except Exception as e:
                            self.node.get_logger().error(f"Error in parameter callback for {param.name}: {e}")

__all__ = ['ParameterEventHandler']
