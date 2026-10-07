from setuptools import setup
import os
from glob import glob

package_name = 'can_manager'

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
    ],
    install_requires=['setuptools', 'python-can'],
    zip_safe=True,
    maintainer='team',
    description='ROS2 - CAN gateway node for Main ECU',
    license='MIT',
    entry_points={
        'console_scripts': [
            'can_manager_node = can_manager.can_manager_node:main',
        ],
    },
)
