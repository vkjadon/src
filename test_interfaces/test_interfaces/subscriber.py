import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from custom_interfaces.msg import Num

class SubNode(Node):

    def __init__(self):
        super().__init__('min_sub')
        self.subscription = self.create_subscription(Num, 'topic', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        data=msg.num
        self.get_logger().info(f'Data Received : {data}')

def main(args=None):
    rclpy.init(args=args)

    try:
        node = SubNode()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)

if __name__ == '__main__':
    main()