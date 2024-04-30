from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='camera_package',
            executable='movement_test',
            name='movement_test',
            output='screen',
        ),
        Node(
            package='camera_package',
            executable='arduino_interface',
            name='arduino_interface',
            output='screen',
        ),

    ])
