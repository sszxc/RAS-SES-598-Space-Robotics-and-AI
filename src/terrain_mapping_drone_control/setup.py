from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'terrain_mapping_drone_control'

setup(
    name=package_name.replace('_', '-'),
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A ROS2 package for executing spiral trajectory with PX4',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest']
    },
    entry_points={
        'console_scripts': [
            'spiral_trajectory = terrain_mapping_drone_control.spiral_trajectory:main',
        ],
    },
) 