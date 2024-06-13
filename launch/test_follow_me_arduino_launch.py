from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        
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
                {'motor_settings_radius': 35},
                {'motor_settings_wheel_distance': 11},
                {'motor_settings_wheel_radius': 3.5},
                {'motor_settings_correction_factor': 1.0},
                {'motor_settings_base_rpm': 100},
                {'enable_movement': True},
                {'enable_servo': False},
            ]
        ),
        Node(
            package='camera_package',
            executable='test_movement_control',
            name='test_movement_control',
            output='screen',
        ),

    ])
