from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='camera_package',
            executable='camera_opencv_loop',
            name='camera_opencv_loop',
            output='screen',
            parameters=[
                {'detector_type': "haarcascade"},
                {'timer_period': 0.5},
                {'optimal_hight_percentage': 75},
            ]
        ),
        Node(
            package='camera_package',
            executable='movement_control',
            name='movement_control',
            output='screen',
            parameters=[
                {'camera_max_winkel_x': 85},
                {'camera_max_winkel_y': 50},
                {'distance_to_person': 200},
                {'hight_of_person': 170},
                {'motor_settings_radius': 25},
                {'motor_settings_wheel_distance': 9},
                {'motor_settings_wheel_radius': 2.3},
                {'motor_settings_correction_factor': 1.0},
                {'motor_settings_base_rpm': 100},
                {'enable_movement': True},
                {'enable_servo': False},
            ]
        ),
        Node(
            package='camera_package',
            executable='camera_streamer',
            name='camera_streamer',
            output='screen',
            parameters=[
                {'ip': "192.168.5.1"},
            ]
        ),
        Node(
            package='ros2_for_waveshare_alphabot2',
            executable='joystick',
            name='joystick',
            output='screen',
        ),
        Node(
            package='ros2_for_waveshare_alphabot2',
            executable='motion',
            name='motion',
            output='screen',
        ),

    ])
