#!/usr/bin/env python3
# =============================================================================
#  mission.launch.py — 임무 전체 구성
# =============================================================================
#  띄우는 것:  sllidar -> 회피(리맵) -> 서보 -> mux -> can_manager
#              + task_manager + rear_camera(캡쳐) + tof(판정)
#
#  ★ 기본값은 '안전한 쪽'이다 ★
#    autostart:=false      스스로 출발하지 않는다
#    with_can:=false       CAN 에 아무것도 내보내지 않는다 (로직만 확인)
#  시연에서는 둘 다 true 로 주고, ★반드시 바퀴 상태를 먼저 확인★한다.
#
#  사용 예
#    # 로직만 (CAN 무송신 · 안전)
#    ros2 launch drain_task_manager mission.launch.py
#
#    # 바퀴 띄우고 실모터까지
#    ros2 launch drain_task_manager mission.launch.py with_can:=true
#
#    # 시연 (바닥. 전원 인가 후 10초 뒤 자동 출발)
#    ros2 launch drain_task_manager mission.launch.py \
#        with_can:=true autostart:=true require_task:=true
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
#  ★ 문자열 인자를 문자열로 강제하는 데 필요하다 ★ 아래 as_str 주석 참조.
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    task_params = os.path.join(
        get_package_share_directory('drain_task_manager'),
        'config', 'task_params.yaml')
    servo_params = os.path.join(
        get_package_share_directory('drain_servoing'),
        'config', 'servo_params.yaml')
    rear_params = os.path.join(
        get_package_share_directory('drain_vision'),
        'config', 'rear_camera_params.yaml')
    #  ★ [2026-08-29] 여기가 비어 있었다 ★ drain_detector 를 파라미터 없이 띄우고 있어서
    #     detector_params.yaml 의 model_path·conf_threshold·class_names 가 전부 무시됐다.
    #     v2 모델로 바꾸면서 발견 — yaml 만 고쳤으면 이 launch 는 옛 모델을 계속 물었다.
    detector_params = os.path.join(
        get_package_share_directory('drain_vision'),
        'config', 'detector_params.yaml')
    tof_params = os.path.join(
        get_package_share_directory('drain_tof'),
        'config', 'tof_params.yaml')
    can_params = os.path.join(
        get_package_share_directory('can_manager'),
        'config', 'can_params.yaml')
    comms_params = os.path.join(
        get_package_share_directory('drain_comms'),
        'config', 'comms_params.yaml')

    #  ★★ [2026-08-29] launch 인자를 문자열 파라미터에 그냥 넘기면 안 된다 ★★
    #  launch 는 넘긴 값을 YAML 1.1 로 추론한다. 그래서 'OFF' 가 **불리언 False** 가 되고
    #  (YAML 1.1 은 on/off/yes/no 도 불리언이다), 문자열을 기대하는 노드가
    #    InvalidParameterTypeException: 'test_verdict' to 'False' of type 'BOOL'
    #  로 **기동하자마자 죽는다.** 2026-08-29 자동 시작을 만들다 실제로 겪었다 —
    #  ★ 죽은 것이 task_manager 라 임무 전체가 시작조차 못 했다 ★
    #  value_type=str 로 추론을 막는다.
    def as_str(cfg):
        return ParameterValue(cfg, value_type=str)

    autostart = LaunchConfiguration('autostart')
    require_task = LaunchConfiguration('require_task')
    with_can = LaunchConfiguration('with_can')
    with_lidar = LaunchConfiguration('with_lidar')
    with_vision = LaunchConfiguration('with_vision')
    with_rear_camera = LaunchConfiguration('with_rear_camera')
    with_tof = LaunchConfiguration('with_tof')
    simulate_tof = LaunchConfiguration('simulate_tof')
    test_verdict = LaunchConfiguration('test_verdict')
    with_comms = LaunchConfiguration('with_comms')

    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='false',
                              description='전원 인가 후 자동 출발 (시연에서만 true)'),
        DeclareLaunchArgument('require_task', default_value='false',
                              description='mux 가 임무 게이트를 필수로 볼지 (바닥에서 true)'),
        DeclareLaunchArgument('with_can', default_value='false',
                              description='can_manager 기동 — true 면 실모터가 돈다'),
        DeclareLaunchArgument('with_lidar', default_value='true'),
        DeclareLaunchArgument('with_vision', default_value='true'),
        #  후방 카메라(C920) 캡쳐 노드. 없으면 TaskManager 가 3초 뒤
        #  '영상 없이' 보고로 넘어간다 — 임무는 멈추지 않는다.
        DeclareLaunchArgument('with_rear_camera', default_value='true'),
        #  TOF 판정 노드. 없으면 TaskManager 가 3초 뒤 UNKNOWN 으로 진행한다.
        DeclareLaunchArgument('with_tof', default_value='true'),
        #  ★ 시험 전용 ★ "3,10,11,10,0,12" 처럼 6개를 주면 TOF 보드 없이
        #  판정 경로를 돌려볼 수 있다. 비어 있으면 실제 /tof/status 를 본다.
        DeclareLaunchArgument('simulate_tof', default_value=''),
        DeclareLaunchArgument('test_verdict', default_value='OFF',
                              description='TOF 노드 없이 판정을 주입 (시험 전용)'),
        #  ★★ 서버 전송 — 기본은 꺼짐이다 ★★
        #  켜면 **실제 서버(MQTT 146.56.110.22 · HTTP data-myonge.yuhkm.kr)로 데이터가 나간다.**
        #  ★ 왜 기본을 끄나 ★ server_bridge 는 `mqtt_client.connect()` 가
        #  **모듈 최상단**이라 브로커가 안 열려 있으면 **import 중에 죽는다.**
        #  기본으로 켜 두면 시연 때 노드 하나가 조용히 빠진다
        #  (test_verdict 가 task_manager 를 죽였던 것과 같은 모양).
        DeclareLaunchArgument('with_comms', default_value='false',
                              description='서버 전송 — ★true 면 실제 서버로 나간다★'),

        #  LiDAR
        #  ★★ [2026-08-29] respawn 을 켠 이유 — 이 노드는 실제로 죽는다 ★★
        #  거칠게 종료된 직후 다시 열면 `SL_RESULT_OPERATION_TIMEOUT` 으로
        #  **기동에 실패하고 프로세스가 통째로 사라진다**(exit 255). 그러면
        #  /scan 이 영영 안 나오고 → /cmd_vel_avoid 없음 → 시동 전 점검이 ABORT.
        #  ★ 전원만 넣고 도는 구성에서는 이게 임무 전체를 죽인다 ★ 사람이 없으니까.
        #  실측: 같은 장치가 몇 초 뒤 재시도하면 **세 번 다 정상 기동**했다.
        #  그래서 '고장'이 아니라 '다시 열면 되는 것'이고, 답은 재시도다.
        #  (task_params.yaml 의 preflight_grace 가 그동안 기다려 준다)
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='sllidar_ros2',
                    executable='sllidar_node',
                    name='sllidar_node',
                    output='screen',
                    condition=IfCondition(with_lidar),
                    respawn=True,
                    respawn_delay=3.0,
                    parameters=[{
                        'channel_type': 'serial',
                        'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                        'serial_baudrate': 115200,
                        'frame_id': 'laser',
                        'inverted': False,
                        'angle_compensate': False
                   }]
             )
        ]
    ),

        #  회피 — ★코드를 고치지 않고 리맵으로 붙인다★
        #  ★ [2026-08-30] 회피 속도를 여기서 넘긴다 ★
        #    그전에는 파라미터를 하나도 안 넘겨 노드 기본값(0.45/2.25/0.35)으로 돌았다.
        #    drain_lidar_avoidance 패키지 소스는 건드리지 않는다(CLAUDE.md 규칙) —
        #    튜닝은 이 자리에서 한다.
        #  ★★ [2026-08-30] 세 값 모두 명령 41rpm 으로 통일했다 (사용자 결정) ★★
        #    "하한을 40으로 두고 41rpm으로 주행하자" — 0.225(60.5rpm)로도 배수구를
        #    지나쳐서 더 내렸다. 문서상 실용 하한 60rpm 아래이므로 끊김 여부를
        #    반드시 눈으로 확인할 것(미해결 3-B). 끊기면 올린다.
        Node(package='drain_lidar_avoidance', executable='obstacle_avoidance_node',
             name='obstacle_avoidance_node', output='screen',
             condition=IfCondition(with_lidar),
             parameters=[{
                 'forward_speed': 0.1524,  # 명령 41.0rpm (원래 0.45=121rpm)
                 #  ★★ [2026-09-06] 회전만 명령 60rpm 으로 올렸다 (사용자 결정) ★★
                 #    원문: "회피로직에서 회전시에만 rpm을 60으로 변경해줘.
                 #           회전시에 움찔거리기만 하고 거의 안 돌아."
                 #    41rpm 은 문서상 실용 하한 60rpm 아래다(미해결 3-B) — 위 08-30
                 #    주석의 "끊기면 올린다"가 회전에서 실제로 걸린 것이다.
                 #    제자리 선회라 v=0 이고 바퀴 속력은 w·L/2 이므로
                 #      w = 2 · (60/269.0) / 0.40 = 1.115 rad/s  -> 좌우 ∓60/±60 rpm
                 #    ★ 전진·후진은 41rpm 그대로 둔다 ★ 접근 속도를 올리면
                 #    배수구를 지나쳐 버린다(08-30 에 그래서 내린 값이다).
                 #  ★★ [2026-09-06 17:40] 사용자가 1.115 → 1.859 로 직접 올렸다 ★★
                 #    (2026-09-10 사용자 확인). 60rpm 에서도 회전이 모자랐던 것으로 보인다.
                 #      1.859 · 0.40 / 2 · 269.0 = 100.0  -> 좌우 ∓100/±100 rpm
                 #    (wheel_separation 0.40 · wheel_radius 0.071 · gear_ratio 2.0 기준)
                 #    max_rpm 300 아래라 클램프 없음. 실제 바퀴는 명령의 절반(약 50rpm)으로 돈다.
                 'turn_speed': 1.859,      # 바퀴 명령 100.0rpm (09-06 1.115=60rpm · 08-30 0.762=41rpm)
                 'reverse_speed': 0.1524,  # 명령 41.0rpm (원래 0.35=94rpm)
             }],
             remappings=[('/cmd_vel', '/cmd_vel_avoid')]),

        #  카메라 + 탐지
        #  ★ [2026-08-30] respawn 을 붙였다 ★ 부팅 직후 USB 카메라가 아직 스트림을
        #  못 여는 창이 있다. 그때 열기에 실패하면 이 노드는 예외로 **죽고 끝난다** —
        #  그러면 /image_raw 발행자가 0개라 탐지도 감시 화면도 영영 안 온다
        #  (2026-08-30 실제로 겪었다: 부팅 자동 시작 3회 연속 같은 자리에서 죽었다).
        #  sllidar_node 와 같은 처방이다. 3초 뒤 다시 띄우면 대개 그 사이에 준비된다.
        Node(package='drain_vision', executable='camera_publisher',
             name='camera_publisher', output='screen',
             respawn=True, respawn_delay=3.0,
             condition=IfCondition(with_vision)),
        Node(package='drain_vision', executable='drain_detector',
             name='drain_detector', output='screen',
             parameters=[detector_params],
             condition=IfCondition(with_vision)),

        #  서보 + mux
        Node(package='drain_servoing', executable='drain_servo_node',
             name='drain_servo_node', output='screen',
             parameters=[servo_params]),
        Node(package='drain_servoing', executable='cmd_mux_node',
             name='cmd_mux_node', output='screen',
             parameters=[servo_params, {'require_task': require_task}]),

        #  후방 카메라 — 캡쳐 서비스만 제공한다. 평소에는 장치를 열지 않는다
        #  (keep_open=false). ★ 두 개 띄우지 말 것 ★ 장치를 못 연 쪽이 먼저
        #  실패로 답하면 캡쳐가 성공해도 실패로 보고된다(2026-08-24에 겪음).
        Node(package='drain_vision', executable='rear_camera_node',
             name='rear_camera_node', output='screen',
             condition=IfCondition(with_rear_camera),
             parameters=[rear_params]),

        #  TOF 판정 — CAN 을 열지 않는다. can_manager 의 /tof/status 만 본다.
        Node(package='drain_tof', executable='tof_node',
             name='tof_node', output='screen',
             condition=IfCondition(with_tof),
             parameters=[tof_params,
                         {'simulate_distances': as_str(simulate_tof)}]),

        #  임무 관리
        Node(package='drain_task_manager', executable='task_manager_node',
             name='task_manager_node', output='screen',
             parameters=[task_params,
                         {'autostart_enable': autostart,
                          'test_verdict': as_str(test_verdict)}]),

        #  CAN — ★이게 true 면 실제로 모터가 돈다★
        Node(package='can_manager', executable='can_manager_node',
             name='can_manager', output='screen',
             condition=IfCondition(with_can),
             parameters=[can_params]),

        #  ── 서버 전송 두 노드 (with_comms:=true 일 때만) ──────────────────
        #  ★ 순서가 있다 ★ 어댑터가 ReportDrain 서비스를 열고, server_bridge 가
        #  그 결과를 토픽으로 받아 MQTT/HTTP 로 낸다. 둘 다 없으면 TaskManager 는
        #  3초 타임아웃 뒤 ~/drain_reports.jsonl 에 sent:false 로 남기고 진행한다.
        #
        #  ★ server_bridge 는 서버 팀 코드다 — 고치지 않는다 ★
        #  이름을 'comms_node' 로 두면 안 된다(어댑터가 그 이름을 쓴다).
        Node(package='drain_comms', executable='comms_adapter_node',
             name='comms_node', output='screen',
             condition=IfCondition(with_comms),
             parameters=[comms_params]),
        Node(package='server_bridge', executable='server_bridge_node01',
             name='server_bridge', output='screen',
             condition=IfCondition(with_comms)),
    ])
