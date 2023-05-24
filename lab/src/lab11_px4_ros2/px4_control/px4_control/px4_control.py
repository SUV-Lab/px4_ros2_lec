import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


class PX4Control(Node):
    def __init__(self):
        super().__init__('px4_control')
        # PX4를 제어하기위해 필요한 토픽들 publish
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,
                                                                      '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint',
                                                                    10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.offboard_setpoint_counter_ = 0
        # 0.5초 주기로 time_callback 호출
        self.timer_callback = self.create_timer(0.5, self.timer_callback)

    # Arm 명령 보내기
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command send')

    # Disarm 명령 보내기
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarm command send')

    # 오프보드 제어 모드를 publish.
    # 이 예제에서는 위치와 고도 제어만 활성화되어 있습니다.
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(int(self.get_clock().now().nanoseconds / 1000))
        self.offboard_control_mode_publisher_.publish(msg)

    # 경로 설정점을 publish
    # 이 예제에서는 5미터 상공에서 Yaw 각도가 180도인 위치에 고정
    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = -3.14
        msg.timestamp = int(int(self.get_clock().now().nanoseconds / 1000))
        self.trajectory_setpoint_publisher_.publish(msg)

    # 차량 명령을 게시합니다.
    # command: 명령 코드 (VehicleCommand 및 MAVLink MAV_CMD 코드와 일치)
    # param1: MAVLink uint16 VEHICLE_CMD 열거형에 정의된 Command parameter 1
    # param2: MAVLink uint16 VEHICLE_CMD 열거형에 정의된 Command parameter 2

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

    def timer_callback(self):
        # 5초 비행 시작
        if self.offboard_setpoint_counter_ == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            self.arm()

        # px4에서 offboard 비행을 위해 주기적으로 pubslih
        self.publish_offboard_control_mode()

        # 20 초뒤부터 앞으로 10미터 이동
        if self.offboard_setpoint_counter_ < 40:
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0)
        else:
            self.publish_trajectory_setpoint(10.0, 0.0, -5.0)

        self.offboard_setpoint_counter_ += 1


def main(args=None):
    print('Starting PX4 control node...')
    rclpy.init(args=args)
    px4_control = PX4Control()
    rclpy.spin(px4_control)
    px4_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()