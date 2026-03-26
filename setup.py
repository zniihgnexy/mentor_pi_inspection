from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'mentor_pi_inspection'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)],
        ),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mentor_pi_inspection',
    maintainer_email='maintainer@example.com',
    description='MentorPi classroom inspection robot package.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_interface = '
            'mentor_pi_inspection.sensor_layer.camera_interface:main',
            'data_recorder = mentor_pi_inspection.data_layer.data_recorder:main',
            'depth_fusion = mentor_pi_inspection.sensor_layer.depth_fusion:main',
            'image_processor = mentor_pi_inspection.vision_layer.image_processor:main',
            'inspection_manager = '
            'mentor_pi_inspection.control_layer.inspection_manager:main',
            'keyboard_teleop = '
            'mentor_pi_inspection.control_layer.keyboard_teleop:main',
            'lidar_interface = mentor_pi_inspection.sensor_layer.lidar_interface:main',
            'motion_controller = '
            'mentor_pi_inspection.navigation_layer.motion_controller:main',
            'path_planner = mentor_pi_inspection.navigation_layer.path_planner:main',
            'power_led_detector = '
            'mentor_pi_inspection.vision_layer.power_led_detector:main',
            'sensor_manager = mentor_pi_inspection.sensor_layer.sensor_manager:main',
            'slam_node = mentor_pi_inspection.navigation_layer.slam_node:main',
        ],
    },
)
