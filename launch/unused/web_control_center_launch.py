from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='camera_package',
            executable='web_control_center',
            name='web_control_center',
            output='screen',
        ),

    ])
