from setuptools import setup
import os
from glob import glob

package_name = 'drain_servoing'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    description='배수로 비주얼 서보잉 + cmd_vel 조정자',
    license='MIT',
    entry_points={
        'console_scripts': [
            'drain_servo_node = drain_servoing.drain_servo_node:main',
            'cmd_mux_node = drain_servoing.cmd_mux_node:main',
        ],
    },
)
