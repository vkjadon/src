import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import time

from custom_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback,
            goal_callback=self.goal_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal....')

        #Set goal request
        order = goal_handle.request.order
 
        #Executing Goal       
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        #Setting Goal Status
        goal_handle.succeed()

        #Creating Result Object
        result = Fibonacci.Result()
        
        result.sequence = feedback_msg.partial_sequence

        return result        
    
    # Callback to handle new goal requests (accept or reject)
    def goal_callback(self, goal_handle):
        #Set goal request
        order = goal_handle.order
        if(order > 10):
            self.get_logger().info('Received goal request...')
            # Accept every incoming goal
            return rclpy.action.GoalResponse.REJECT
        else:
            return rclpy.action.GoalResponse.ACCEPT
    
    # Callback to handle goal cancellations
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received request to cancel goal')
        return rclpy.action.CancelResponse.ACCEPT
    
def main():
    rclpy.init()

    try:
        node = FibonacciActionServer()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        print("Test interrupted by user. Exiting...")

if __name__ == '__main__':
    main()