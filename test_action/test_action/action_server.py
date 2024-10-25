import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from custom_interfaces.action import CountAction  # Assuming the action is built

class MyActionServer(Node):

    def __init__(self):
        super().__init__('my_action_server')
        
        # Create an action server
        self._action_server = ActionServer(
            self,
            CountAction,
            'my_action',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    # Callback to handle new goal requests (accept or reject)
    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request with count: %d' % goal_request.goal_count)
        # Accept every incoming goal
        return rclpy.action.GoalResponse.ACCEPT

    # Callback to handle goal cancellations
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received request to cancel goal')
        return rclpy.action.CancelResponse.ACCEPT

    # Callback to execute the goal
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = CountAction.Result()  # Initialize result

        for i in range(1, goal_handle.request.goal_count + 1):
            if goal_handle.is_cancel_requested():
                self.get_logger().info('Goal canceled')
                goal_handle.canceled()
                result.result_message = 'Goal canceled midway'
                return result

            # Simulate progress (1 second delay)
            time.sleep(1)
            feedback_msg = CountAction.Feedback()
            feedback_msg.current_progress = int((i / goal_handle.request.goal_count) * 100)
            goal_handle.publish_feedback(feedback_msg)

        # Mark the goal as successful
        goal_handle.succeed()
        result.result_message = 'Goal completed successfully'
        self.get_logger().info(result.result_message)
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = MyActionServer()
    rclpy.spin(action_server)

    action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
