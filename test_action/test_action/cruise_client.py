#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from custom_interfaces.action import CruiseSpeed

class CruiseActionClientNode(Node):

    def __init__(self):
        super().__init__('cruise_node')

        self.cruse_action_client=ActionClient(self, CruiseSpeed, "cruise_speed")
        

    def send_goal(self, cruise_speed, cruise_step):
        
        self.cruse_action_client.wait_for_server()

        goal = CruiseSpeed.Goal()

        goal._cruise_speed = cruise_speed
        goal.cruise_step = cruise_step

        self.get_logger().info("Sending Goal to Server")
        self.cruse_action_client.send_goal_async(goal)

def main():
    rclpy.init()

    try:
        node = CruiseActionClientNode()
        node.send_goal(120,5)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        print("Test interrupted by user. Exiting...")

if __name__ == '__main__':
    main()