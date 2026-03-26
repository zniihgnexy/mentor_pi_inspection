from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mentor_pi_inspection'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@manchester.ac.uk',
    description='Computer Lab Inspection Robot based on MentorPi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_manager = mentor_pi_inspection.sensor_layer.sensor_manager:main',
            'lidar_interface = mentor_pi_inspection.sensor_layer.lidar_interface:main',
            'camera_interface = mentor_pi_inspection.sensor_layer.camera_interface:main',
            'slam_node = mentor_pi_inspection.navigation_layer.slam_node:main',
            'path_planner = mentor_pi_inspection.navigation_layer.path_planner:main',
            'motion_controller = mentor_pi_inspection.navigation_layer.motion_controller:main',
            'power_led_detector = mentor_pi_inspection.vision_layer.power_led_detector:main',
            'image_processor = mentor_pi_inspection.vision_layer.image_processor:main',
            'inspection_manager = mentor_pi_inspection.control_layer.inspection_manager:main',
            'keyboard_teleop = mentor_pi_inspection.control_layer.keyboard_teleop:main',
            'scheduler = mentor_pi_inspection.control_layer.scheduler:main',
            'data_recorder = mentor_pi_inspection.data_layer.data_recorder:main',
            'depth_fusion = mentor_pi_inspection.sensor_layer.depth_fusion:main',
            'evaluation = mentor_pi_inspection.test.test_evaluation:main',
        ],
    },
)
