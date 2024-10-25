import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServerNode(Node):

    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.callback_add_two_ints)
        self.get_logger().info("Service Server Started !! ")

    def callback_add_two_ints(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request a:{request.a} b:{request.b} Response is {response.sum}')

        return response

def main():
    rclpy.init()

    try:
        node = ServerNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("Test interrupted by user. Exiting...")

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()