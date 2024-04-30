from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='camera_package',
            executable='camera_opencv_loop2',
            name='camera_opencv_loop2',
            output='screen',
        ),
        Node(
            package='camera_package',
            executable='movement_control',
            name='movement_control',
            output='screen',
        ),
        Node(
            package='ros2_for_waveshare_alphabot2',
            executable='joystick',
            name='joystick',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='camera_package',
            executable='camera_streamer',
            name='camera_streamer',
            output='screen',
        ),
        Node(
            package='camera_package',
            executable='arduino_interface',
            name='arduino_interface',
            output='screen',
        ),

    ])
