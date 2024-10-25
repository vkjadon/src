from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dc_motor_control',
            executable='motor_pid_exe',  
            name='motor_pid',
            parameters=['/home/rospi/ros_ws/src/dc_motor_control/config/params.yaml'],  
            output='screen' 
        )
    ])