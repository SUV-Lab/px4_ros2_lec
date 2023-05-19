from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('service')
        self.addsrv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.subsrv = self.create_service(AddTwoInts, 'sub_two_ints', self.sub_two_ints_callback)
        self.mulsrv = self.create_service(AddTwoInts, 'mul_two_ints', self.mul_two_ints_callback)
        self.divsrv = self.create_service(AddTwoInts, 'div_two_ints', self.div_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'add : {request.a} + {request.b} = {response.sum}')
        return response

    def sub_two_ints_callback(self, request, response):
        response.sum = request.a - request.b
        self.get_logger().info(f'sub : {request.a} - {request.b} = {response.sum}')
        return response

    def mul_two_ints_callback(self, request, response):
        response.sum = request.a * request.b
        self.get_logger().info(f'mul : {request.a} * {request.b} = {response.sum}')
        return response

    def div_two_ints_callback(self, request, response):
        response.sum = request.a // request.b
        self.get_logger().info(f'div : {request.a} / {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()