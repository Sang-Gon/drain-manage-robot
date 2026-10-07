"""카메라 노드만 실행하는 launch 파일.

Hailo 가속기가 없어도 동작한다. 배선/카메라 확인용.

사용법:
    ros2 launch drain_vision camera.launch.py
    ros2 launch drain_vision camera.launch.py camera_device:=2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('drain_vision')
    params = os.path.join(share, 'config', 'camera_params.yaml')

    camera_device = LaunchConfiguration('camera_device')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_device', default_value='0',
            description='/dev/videoN 의 N'),

        Node(
            package='drain_vision',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
            parameters=[params, {'camera_device': camera_device}],
            emulate_tty=True,   # 로그 색상 유지
        ),
    ])
