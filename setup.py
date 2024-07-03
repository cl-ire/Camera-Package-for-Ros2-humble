from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[py]*'))),
        (os.path.join('share', package_name, 'camera_package/views'), glob(os.path.join('camera_package/views', '*.[html]*'))),
        (os.path.join('share', package_name, 'camera_package/static'), glob(os.path.join('camera_package/static', '*.[js]*'))),
        (os.path.join('share', package_name, 'camera_package/static'), glob(os.path.join('camera_package/static', '*.[css]*'))),
        (os.path.join('share', package_name, 'opencv/data'), glob(os.path.join('opencv/data', '*.[xml]*'))),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Claire Schubert',
    maintainer_email='ubuntu@todo.todo',
    description='Camera Subscriber for the topic /image_raw form the v4l2_camera_node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_test = camera_package.camera_subscriber_test:main',
            'camera_opencv = camera_package.camera_opencv_node:main',
            'camera_opencv_loop = camera_package.camera_opencv_loop_node:main',
            'movement_control = camera_package.movement_control_node:main',
            'camera_streamer = camera_package.camera_streamer_node:main',
            'web_control_center = camera_package.web_control_center_node:main',
            'camera_save_image = camera_package.camera_save_image_node:main', 
            'arduino_interface = camera_package.arduino_interface_node:main',
            'movement_test = camera_package.movement_test_node:main',
            'test_movement_control = camera_package.test_movement_control_node:main',         
        ],
    },
)
