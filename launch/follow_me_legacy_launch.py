from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='camera_package',
            executable='camera_opencv',
            name='camera_opencv',
            output='screen',
        ),
        Node(
            package='camera_package',
            executable='movement_control',
            name='movement_control',
            output='screen',
            parameters=[
                {'camera_max_winkel_x': 90},
                {'camera_max_winkel_y': 50},
                {'distance_to_person': 200},
                {'hight_of_person': 170},
                {'motor_settings_radius': 25},
                {'motor_settings_wheel_distance': 9},
                {'motor_settings_wheel_radius': 2.3},
                {'motor_settings_correction_factor': 1},
                {'motor_settings_base_rpm': 50},
                {'enable_movement': False},
            ]
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

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            output='screen',
            parameters=[
                {'image_size': [640,480]},                     
            ],
        ),

        Node(
            package='camera_package',
            executable='camera_streamer',
            name='camera_streamer',
            output='screen',
        ),

    ])
