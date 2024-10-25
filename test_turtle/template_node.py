#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class Node(Node):

    def __init__(self):
        super().__init__('service_server')
        self.get_logger().info("Node Started !! ")

        self.callback_process()

    def callback_process(self):        
        self.get_logger().info("Callback Function")

def main():
    rclpy.init()

    try:
        node = Node()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        print("Test interrupted by user. Exiting...")

if __name__ == '__main__':
    main()