import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class Circle(Node):
    def __init__(self):
        super().__init__('circle')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.vel_x = 2.0  
        self.omega_z = 2.0
        
        # Use v = w x r for direction of rotation and radius

        self.radius = self.vel_x / self.omega_z
        self.get_logger().info(f'Radius of the circle: {self.radius:.2f} units')

        # Calculate the time for circle based on linear and angular velocity
        self.circle_time = (2 * math.pi) / self.omega_z
        self.get_logger().info(f'Time for circle: {self.circle_time:.2f} seconds')

        # Function to move the turtle in a circle
        self.draw_circle()
    
    def draw_circle(self):
        
        # Create a message to draw a circle
        # The message is to be published on '/turtle1/cmd_vel'
        # This topic accepts the data structure of 'Twist'
        # 'Twist' is available in 'geometry_msgs' interface package
        
        twist = Twist()
        twist.linear.x = self.vel_x
        twist.angular.z = self.omega_z

        # Publish the velocity command for the duration of one circle
        
        # seconds = self.get_clock().now().to_msg().sec
        # nanoseconds = self.get_clock().now().to_msg().nanosec
        # start_time = seconds + nanoseconds/1e9
        start_time = self.get_clock().now().nanoseconds/1e9
        current_time = start_time
        
        '''
        to_msg() converts the time into a ROS 2 builtin_interfaces.msg.Time message, which contains two fields: sec (the full seconds) and nanosec (the fractional part in nanoseconds)
        seconds = self.get_clock().now().to_msg().sec
        nanoseconds = self.get_clock().now().to_msg().nanosec
        '''
        
        while current_time - start_time < self.circle_time:
            self.publisher.publish(twist)
            
            current_time = self.get_clock().now().nanoseconds/1e9
            # self.get_logger().info(f'Time Elapsed : {current_time:.2f} nanoseconds')

        # Stop the turtle
        self.publisher.publish(Twist())
        self.get_logger().info(f'Completed one circle in {(current_time - start_time):.2f} seconds.')

def main(args=None):
    rclpy.init(args=args)
    try:
        node = Circle()
        rclpy.spin(node)
    except :
        rclpy.shutdown()

if __name__ == '__main__':
    main()
