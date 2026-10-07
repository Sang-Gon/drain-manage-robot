from setuptools import setup
import os
from glob import glob

package_name = 'drain_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.hef')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    description='Camera + Hailo YOLOv11s drain detection nodes for Main ECU',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_publisher = drain_vision.camera_publisher:main',
            'drain_detector = drain_vision.drain_detector:main',
            'rear_camera_node = drain_vision.rear_camera_node:main',
        ],
    },
)
