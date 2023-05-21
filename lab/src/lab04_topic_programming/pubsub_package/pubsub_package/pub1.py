import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class Publisher1(Node):
     def __init__(self):
         super().__init__('pub1')
         qos_profile = QoSProfile(depth=10)
         self.publisher1 = self.create_publisher(String, 'pub1/msg', qos_profile)
         self.timer = self.create_timer(1, self.publish_msg)

     def publish_msg(self):
         msg = String()
         msg.data = 'Hello World'
         self.publisher1.publish(msg)
         self.get_logger().info('Published message: {0}'.format(msg.data))

def main(args=None):
     rclpy.init(args=args)
     node = Publisher1()
     try:
         rclpy.spin(node)
     except KeyboardInterrupt:
         node.get_logger().info('Keyboard Interrupt (SIGINT)')
     finally:
         node.destroy_node()
         rclpy.shutdown()

if __name__ == '__main__':
    main()