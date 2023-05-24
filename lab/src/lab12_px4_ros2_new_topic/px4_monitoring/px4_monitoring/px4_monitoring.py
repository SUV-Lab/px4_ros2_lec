import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSPresetProfiles, DurabilityPolicy


from px4_msgs.msg import Monitoring


class PX4Monitoring(Node):

    def __init__(self):
        super().__init__('px4_monitoring')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.subscription = self.create_subscription(
            Monitoring,
            '/fmu/out/monitoring',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print(f'[ PX4 Monitoring ]\n X : {msg.x:.3f} \tY : {msg.y:.3f} \tZ : {msg.z:.3f}')


def main(args=None):
    rclpy.init(args=args)

    px4_monitoring = PX4Monitoring()

    rclpy.spin(px4_monitoring)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    px4_monitoring.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()