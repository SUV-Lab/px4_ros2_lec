import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int64

class Publisher2(Node):
     def __init__(self):
         super().__init__('pub2')
         qos_profile = QoSProfile(depth=10)
         self.publisher2 = self.create_publisher(Int64, 'pub2/msg', qos_profile)
         self.timer = self.create_timer(1, self.publish_msg)
         self.count = 0

     def publish_msg(self):
         msg = Int64()
         msg.data = self.count
         self.publisher2.publish(msg)
         self.get_logger().info('Published message: {0}'.format(msg.data))
         self.count += 1

def main(args=None):
     rclpy.init(args=args)
     node = Publisher2()
     try:
         rclpy.spin(node)
     except KeyboardInterrupt:
         node.get_logger().info('Keyboard Interrupt (SIGINT)')
     finally:
         node.destroy_node()
         rclpy.shutdown()

if __name__ == '__main__':
    main()