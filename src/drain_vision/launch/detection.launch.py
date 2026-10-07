"""카메라 + Hailo 탐지 노드를 함께 실행하는 launch 파일.

    camera_publisher ──/image_raw──> drain_detector ──/detections/image──>

★ drain_detector 는 Hailo 가속기(PCIe)가 있어야 뜬다.
  없는 상태로 실행하면 hailo_platform import 에서 안내 메시지와 함께 죽는다.
  그때는 camera.launch.py 로 카메라만 확인할 것.

사용법:
    ros2 launch drain_vision detection.launch.py
    ros2 launch drain_vision detection.launch.py conf_threshold:=0.5
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('drain_vision')
    camera_params = os.path.join(share, 'config', 'camera_params.yaml')
    detector_params = os.path.join(share, 'config', 'detector_params.yaml')

    conf = LaunchConfiguration('conf_threshold')

    return LaunchDescription([
        DeclareLaunchArgument(
            'conf_threshold', default_value='0.3',
            description='탐지 신뢰도 하한 (0.0~1.0)'),

        Node(
            package='drain_vision',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
            parameters=[camera_params],
            emulate_tty=True,
        ),
        Node(
            package='drain_vision',
            executable='drain_detector',
            name='drain_detector',
            output='screen',
            parameters=[detector_params, {'conf_threshold': conf}],
            emulate_tty=True,
        ),
    ])
