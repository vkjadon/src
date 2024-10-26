#Commented
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from custom_interfaces.msg import Num

class PubNode(Node):

    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(Num, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Num()
        msg.num = self.i
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.num}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    try:
        node = PubNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == '__main__':
    main()