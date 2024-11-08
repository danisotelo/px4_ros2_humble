#!/usr/bin/env python3

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_following'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Marker file for colcon package indexing
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Package metadata file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include all RViz configuration files
        (os.path.join('share', package_name, 'resource/rviz'), glob('resource/rviz/*.rviz')),
        # Include all config files in the config directory
        (os.path.join('share', package_name, 'resource/config'), glob('resource/config/*.yaml')),
        (os.path.join('share', package_name, 'resource/models/camera'), glob('resource/models/camera/*.config')),
        (os.path.join('share', package_name, 'resource/models/ur5e_camera'), glob('resource/models/ur5e_camera/*.config')),
        # Include the SDF model file
        (os.path.join('share', package_name, 'resource/models/ur5e_camera'), glob('resource/models/ur5e_camera/*.sdf')),
        (os.path.join('share', package_name, 'resource/models/camera'), glob('resource/models/camera/*.sdf')),
        # Include all meshes
        (os.path.join('share', package_name, 'resource/models/ur5e_camera/meshes/d435'), glob('resource/models/ur5e_camera/meshes/d435/*.stl')),
        (os.path.join('share', package_name, 'resource/models/ur5e_camera/meshes/robotiq_85_gripper/collision'), glob('resource/models/ur5e_camera/meshes/robotiq_85_gripper/collision/*.stl')),
        (os.path.join('share', package_name, 'resource/models/ur5e_camera/meshes/robotiq_85_gripper/visual'), glob('resource/models/ur5e_camera/meshes/robotiq_85_gripper/visual/*.dae')),
        (os.path.join('share', package_name, 'resource/models/ur5e_camera/meshes/ur5e/collision'), glob('resource/models/ur5e_camera/meshes/ur5e/collision/*.stl')),
        (os.path.join('share', package_name, 'resource/models/ur5e_camera/meshes/ur5e/visual'), glob('resource/models/ur5e_camera/meshes/ur5e/visual/*.dae')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danisotelo',
    maintainer_email='danisotacam@gmail.com',
    description='Robotic arm drone following',
    license='BSD-3',
    entry_points={
        'console_scripts': [
                'drone_control = drone_following.drone_control:main',
                'visualizer = drone_following.visualizer:main',
                'aruco_detector = drone_following.aruco_detector:main',
                'arm_control = drone_following.arm_control:main'
        ],
    },
)
