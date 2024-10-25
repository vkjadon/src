import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import signal 

class MotorControlRPMNode(Node):
    def __init__(self):
        super().__init__('motor_control_rpm_node')

        # Motor and encoder pins
        self.motor_pwm_pin = 18  
        self.motor_in1_pin = 17  
        self.motor_in2_pin = 22
        self.encoder_pin = 27  
        
        #setup pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.motor_pwm_pin, GPIO.OUT)
        GPIO.setup(self.motor_in1_pin, GPIO.OUT)
        GPIO.setup(self.motor_in2_pin, GPIO.OUT)
        GPIO.setup(self.encoder_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Set up PWM signal at 1000 Hz on the PWM pin 
        self.pwm = GPIO.PWM(self.motor_pwm_pin, 1000)
        self.pwm.start(30) 
        
        # Variables for calculating RPM
        self.pulse_count = 0
        self.ppr = 1700  
        self.gear_ratio = 1
        
        # Set up interrupt for encoder pulses
        GPIO.add_event_detect(self.encoder_pin, GPIO.FALLING, callback=self.encoder_callback)

        # Subscriber to listen for duty cycle commands
        self.subscriber = self.create_subscription(Float32, 'duty_cycle', self.duty_cycle_callback, 1)
        
        # Publisher to publish motor_RPM
        self.rpm_publisher = self.create_publisher(Float32, 'motor_rpm', 1)

        # Timer to calculate and publish RPM every second
        self.timer = self.create_timer(1.0, self.calculate_and_publish_rpm)

    def duty_cycle_callback(self, msg):
        duty_cycle = msg.data

        if duty_cycle > 0:
            # Forward direction
            GPIO.output(self.motor_in1_pin, GPIO.HIGH)
            GPIO.output(self.motor_in2_pin, GPIO.LOW)
        elif duty_cycle < 0:
            # Reverse direction
            GPIO.output(self.motor_in1_pin, GPIO.LOW)
            GPIO.output(self.motor_in2_pin, GPIO.HIGH)
        else:
            # Stop the motor
            GPIO.output(self.motor_in1_pin, GPIO.LOW)
            GPIO.output(self.motor_in2_pin, GPIO.LOW)

        # absolute value of speed, capped at 100
        self.pwm.ChangeDutyCycle(min(abs(duty_cycle), 100))

    def encoder_callback(self, channel):
       
        self.pulse_count += 1

    def calculate_and_publish_rpm(self):     
        rpm = (self.pulse_count / self.ppr) / (self.gear_ratio / 60.0)
        # Publish the calculated RPM
        rpm_msg = Float32()
        rpm_msg.data = rpm
        self.rpm_publisher.publish(rpm_msg)
        self.get_logger().info(f'motor_RPM: {rpm}')
        self.pulse_count = 0


    def cleanup_gpio(self):
        self.get_logger().info('Cleaning up GPIO...')
        GPIO.cleanup()  

def main(args=None):
    rclpy.init(args=args)
    motor_control_rpm_node = MotorControlRPMNode()

    def handle_shutdown_signal(signum, frame):
        motor_control_rpm_node.get_logger().info('Ctrl+C received. Shutting down...')
        motor_control_rpm_node.cleanup_gpio()  # Cleanup GPIO
        motor_control_rpm_node.destroy_node()  # Destroy the ROS node
        rclpy.shutdown()  # Shutdown ROS 2 system

    # Register signal handler for SIGINT (Ctrl+C)
    signal.signal(signal.SIGINT, handle_shutdown_signal)
    try:
        rclpy.spin(motor_control_rpm_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Check if rclpy is still running before shutting down
            motor_control_rpm_node.get_logger().info('Shutting down motor control node...')
            motor_control_rpm_node.cleanup_gpio()  # Clean up GPIO resources
            motor_control_rpm_node.destroy_node()  # Destroy the node
            rclpy.shutdown()  # Shutdown ROS 2 properly
        

if __name__ == '__main__':
    main()

