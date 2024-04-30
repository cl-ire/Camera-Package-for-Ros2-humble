from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='camera_package',
            executable='camera_opencv_loop',
            name='camera_opencv_loop',
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
            executable='pan_tilt',
            name='camera_pan_tilt_node',
            output='screen',
        ),

        Node(
            package='ros2_for_waveshare_alphabot2',
            executable='joystick',
            name='joystick',
            output='screen',
            emulate_tty=True,
        ),

    ])
