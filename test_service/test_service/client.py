import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_server')
        self.client = self.create_client(AddTwoInts, '/add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        
        self.send_request(25, 17)

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.get_logger().info(f'Requesting to Sum {request.a} and {request.b}')
        
        # Call the service asynchronously
        future = self.client.call_async(request)
        future.add_done_callback(self.callback_response)

    def callback_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Response {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main():

    rclpy.init()
    try:
        node = ClientNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("Test interrupted by user. Exiting...")

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()