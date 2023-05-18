import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Int64

class Subscriber(Node):
     def __init__(self):
         super().__init__('sub')
         qos_profile = QoSProfile(depth=10)
         self.helloworld_subscriber = self.create_subscription(
             String,
             'pub1/msg',
             self.subscribe_topic_message,
             qos_profile)

         self.helloworld_subscriber2 = self.create_subscription(
             Int64,
             'pub2/msg',
             self.subscribe_topic_message2,
             qos_profile)

     def subscribe_topic_message(self, msg):
         self.get_logger().info('Received message from pub1/msg : {0}'.format(msg.data))

     def subscribe_topic_message2(self, msg):
         self.get_logger().info('Received message from pub2/msg : {0}'.format(msg.data))

def main(args=None):
     rclpy.init(args=args)
     node = Subscriber()
     try:
        rclpy.spin(node)
     except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
     finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()