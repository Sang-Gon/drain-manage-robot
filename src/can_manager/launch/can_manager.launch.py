"""
can_manager 노드 실행 launch 파일.

사용법:
    ros2 launch can_manager can_manager.launch.py

    # 가상 CAN으로 테스트할 때
    ros2 launch can_manager can_manager.launch.py \
        --ros-args -p can_interface:=vcan0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('can_manager'),
        'config', 'can_params.yaml')

    return LaunchDescription([
        Node(
            package='can_manager',
            executable='can_manager_node',
            name='can_manager',
            output='screen',
            parameters=[params],
            emulate_tty=True,   # 로그 색상 유지
        ),
    ])
