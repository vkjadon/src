from launch import LaunchDescription
from launch_ros.actions import Node # type: ignore

def generate_launch_description():
    return LaunchDescription([
        # Publisher node
        Node(
            package='test_topic',   
            executable='pub_exe',   
        ),
        # Subscriber node
        Node(
            package='test_topic',   
            executable='sub_exe',   
        ),
    ])
