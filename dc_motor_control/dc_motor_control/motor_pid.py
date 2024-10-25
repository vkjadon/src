import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
class MotorDutyCycle(Node):
    def __init__(self):
        super().__init__('motor_pid')

        self.rpm_subscriber = self.create_subscription( Float32, 'motor_rpm', self.rpm_callback, 10)

        self.duty_cycle_publisher = self.create_publisher(Float32, 'duty_cycle', 1)

        self.declare_parameter("duty_cycle", 20.0)
        self.declare_parameter("desired_rpm", 50.0)

        self.declare_parameter("kp", 2.0)
        self.declare_parameter("kd", 0.1)

        self.duty_cycle=self.get_parameter("duty_cycle").value

        self.desired_rpm=self.get_parameter("desired_rpm").value

        self.kp=self.get_parameter("kp").value
        self.kd=self.get_parameter("kd").value

        self.start_time=self.get_clock().now().nanoseconds


    def rpm_callback(self, msg):

        self.current_time=self.get_clock().now().nanoseconds
        self.motor_rpm = msg.data
        self.get_logger().info(f'Received motor RPM: {self.motor_rpm}, Desired RPM:{self.desired_rpm}')
        
        self.error=(self.desired_rpm - self.motor_rpm)
        self.duration=(self.current_time-self.start_time)/1e9

        self.duty_cycle=self.duty_cycle + self.kp* self.error + self.kd * self.error/self.duration
        
        if self.duty_cycle > 90.0:
            self.duty_cycle=90.0
        elif self.duty_cycle < 10.0:
            self.duty_cycle=10.0

        self.get_logger().info(f'Calculated Duty Cycle: {self.duty_cycle}% error {self.error}')

        self.duty_cycle_msg = Float32()
        self.duty_cycle_msg.data = self.duty_cycle
        self.duty_cycle_publisher.publish(self.duty_cycle_msg)
        self.get_logger().info(f'Setting Duty Cycle: {self.duty_cycle_msg}% Duration {self.duration} s')

        self.start_time=self.current_time

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
