from launch import LaunchDescription
from launch_ros.actions import Node # type: ignore

def generate_launch_description():
    ld=LaunchDescription(
        [
        # Publisher node
            Node(
            package='test_topic',   
            executable='pub_exe',   #
            #Comment the following and launch, Try different name
            name="publisher_name",  
            output='screen',
            )
        ]
    )
    return ld
