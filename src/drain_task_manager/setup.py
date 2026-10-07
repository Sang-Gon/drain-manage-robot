from setuptools import setup
import os
from glob import glob

package_name = 'drain_task_manager'

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
    description='임무 단계 스위칭 노드',
    license='MIT',
    entry_points={
        'console_scripts': [
            'task_manager_node = drain_task_manager.task_manager_node:main',
        ],
    },
)
