import rclpy
from rclpy.node import Node

class ParamNode(Node):
    def __init__(self):
        super().__init__('param_node')
        self.declare_parameter('param1', 'world')
        self.timer = self.create_timer(1, self.timer_callback)


    def timer_callback(self):
        my_param = self.get_parameter('param1').get_parameter_value().string_value
        self.get_logger().info(f'param1: {my_param}')


def main():
    rclpy.init()
    node = ParamNode()
    rclpy.spin(node)