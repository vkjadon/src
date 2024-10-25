import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from custom_interfaces.action import CountAction  # Assuming the action is built

class MyActionClient(Node):

    def __init__(self):
        super().__init__('my_action_client')
        # Create an action client
        self._action_client = ActionClient(self, CountAction, 'my_action')

    def send_goal(self, goal_count):
        # Wait until the action server is ready
        self._action_client.wait_for_server()

        goal_msg = CountAction.Goal()
        goal_msg.goal_count = goal_count

        self.get_logger().info('Sending goal to count to %d' % goal_count)

        # Send goal to the action server and set up callbacks for result and feedback
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by the server')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('Received feedback: %d%% progress' % feedback_msg.current_progress)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: %s' % result.result_message)

def main(args=None):
    rclpy.init(args=args)
    action_client = MyActionClient()

    # Send a goal of 10 (count to 10)
    action_client.send_goal(10)

    rclpy.spin(action_client)

    action_client.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
