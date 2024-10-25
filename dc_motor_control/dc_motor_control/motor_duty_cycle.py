import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class MotorDutyCycle(Node):
    def __init__(self):
        super().__init__('motor_duty_cycle')

        self.rpm_subscriber = self.create_subscription( Float32, 'motor_rpm', self.rpm_callback, 1)
        self.duty_cycle_publisher = self.create_publisher(Float32, 'duty_cycle', 1)
        self.declare_parameter("duty_cycle", 50.0)
        self.declare_parameter("desired_rpm", 25.0)
        self.declare_parameter("kp", 1.0)

        self.duty_cycle=self.get_parameter("duty_cycle").value
    def rpm_callback(self, msg):
        motor_rpm = msg.data
        self.get_logger().info(f'Received motor RPM: {motor_rpm}')
        
    # def set_motor_duty_cycle(self, msg):
        duty_cycle_msg = Float32()
        duty_cycle_msg.data = self.duty_cycle
        self.duty_cycle_publisher.publish(duty_cycle_msg)
        self.get_logger().info(f'Setting Duty Cycle: {duty_cycle_msg}%')

def main(args=None):
    rclpy.init(args=args)
    motor_duty_cycle = MotorDutyCycle()

    try:
        rclpy.spin(motor_duty_cycle)
    except KeyboardInterrupt:
        pass
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
