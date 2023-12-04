from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path

def generate_launch_description():
    return LaunchDescription([
        LogInfo(
            action=LogInfo.LogInfo.DEFAULT,
            msg="Launching CameraOpencv node..."
        ),
        Node(
            package='camera_package',  # Replace with the actual package name
            executable='camera_opencv',  # Replace with the actual executable name
            name='camera_opencv_node',
            output='screen',
            parameters=[
                {'distance': 0.0},
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([Ros2_for_Waveshare_Alphabot2, 'Alphabot2_standard_launch.py']),
        ),
    ])
