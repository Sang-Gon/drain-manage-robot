# =============================================================================
#  servoing.launch.py — 서보잉 + 조정자 + (선택) 회피
# =============================================================================
#  ★ can_manager 는 여기서 띄우지 않는다 ★
#    can_manager 는 기동 즉시 0x200 을 100ms 주기로 쏘기 시작한다.
#    모터가 도는 노드는 사람이 바퀴 상태를 확인하고 따로 띄우는 편이 안전하다.
#
#  카메라/탐지기도 여기서 띄우지 않는다 — drain_vision 의 detection.launch.py 를 쓸 것.
#
#  사용:
#    ros2 launch drain_servoing servoing.launch.py
#    ros2 launch drain_servoing servoing.launch.py with_avoidance:=false   # 벤치용
#
#  회피 노드는 **리맵으로** 붙인다. 회피 노드 코드는 손대지 않는다.
#    -r /cmd_vel:=/cmd_vel_avoid
# =============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('drain_servoing'), 'config', 'servo_params.yaml')

    with_avoidance = LaunchConfiguration('with_avoidance')

    return LaunchDescription([
        DeclareLaunchArgument(
            'with_avoidance', default_value='true',
            description='회피 노드를 함께 띄운다. false 면 서보 단독 — 바퀴를 띄운 상태에서만.'),

        Node(package='drain_servoing', executable='drain_servo_node',
             name='drain_servo_node', parameters=[params], output='screen'),

        Node(package='drain_servoing', executable='cmd_mux_node',
             name='cmd_mux_node', parameters=[params], output='screen'),

        Node(package='drain_lidar_avoidance', executable='obstacle_avoidance_node',
             name='obstacle_avoidance_node', output='screen',
             condition=IfCondition(with_avoidance),
             remappings=[('/cmd_vel', '/cmd_vel_avoid')]),
    ])
