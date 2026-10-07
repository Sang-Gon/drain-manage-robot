#!/usr/bin/env python3
# =============================================================================
#  task_manager_node — 임무 단계를 스위칭한다
# =============================================================================
#  이 노드는 **주행하지 않는다.** 주행은 회피·서보·mux 가, CAN 은 can_manager 가,
#  판정은 TOF 노드가, 전송은 통신 노드가 한다.
#  여기가 정하는 것은 **지금이 어느 단계인가**와 **그 단계가 끝났는가** 둘뿐이다.
#
#  ── 임무 흐름 ──
#    BOOT_WAIT(10s·점검) -> DRIVING -> SETTLE(5s 정지) -> MOUNTING -> MEASURING -> ADVANCING
#                                                                   |
#                                     DONE <- REPORTING <- CAPTURING
#
#    MOUNTING  서보가 선 자리는 **배수구 앞**이지 위가 아니다. 그 위로 n 만큼
#              더 들어간다. n(mount_distance)은 실측으로 정한다.
#    ADVANCING 50cm 전진. **판정 결과와 무관하게 항상 지난다** —
#              청소가 필요하든 아니든 배수구 사진은 남긴다(2026-08-24 사용자 결정).
#
#  ── /cmd_vel 을 발행하지 않는다 ──
#    CLAUDE.md 의 "**/cmd_vel 발행자는 cmd_mux_node 하나뿐이다**"를 지킨다.
#    이 노드가 내는 것은 **허가**다:
#        /task/allow_drive   Bool   달려도 되는가
#        /task/servo_enable  Bool   서보가 통제권을 가져도 되는가
#    딱 하나 예외가 **/cmd_vel_task** 인데, 이것도 mux 를 거친다.
#    게이트만으로는 "50cm 전진" 같은 정밀 동작을 만들 수 없어서 둔 경로다.
#
#  ── ★ 이 노드의 정지는 '임무 정지'이지 '안전 정지'가 아니다 ★ ──
#    안전은 여전히 3층이 담당한다:
#      cmd_vel 두절 0.3s -> motor_status 두절 1.2s -> ECU RX 워치독 0.9~1.7s
#    **이 노드가 죽어도 안전이 유지되어야 한다.** 그래서 게이트의 페일세이프를
#    이렇게 잡았다:
#      · allow_drive  : mux 가 require_task=true 면 두절 시 정지
#      · servo_enable : 두절 시 서보만 죽고 **회피는 계속 산다**
#        -> 이 노드가 죽어도 로봇이 장애물에 박지 않는다
#
#  ── ★ ADVANCING 중 회피가 끼어들면 '거리를 포기'한다 ★ ──
#    mux 우선순위에서 회피(②)가 TASK(③)보다 위다. 전진 도중 장애물이 나타나면
#    회피가 통제권을 가져가고 로봇은 50cm 를 채우지 못한다.
#    /odom 을 못 쓰므로(미해결 15번) **얼마나 갔는지 알 방법도 없다.**
#    그래서 남은 거리를 이어가지 않고 **전진을 실패로 처리**한다.
#    다만 판정(MEASURING)은 전진보다 **먼저** 끝나 있으므로
#    잃는 것은 사진 한 장뿐이다. 임무를 통째로 실패시키지 않고
#    "판정 + 캡쳐 실패 사유"를 서버로 보낸다.
# =============================================================================

import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from robot_interfaces.msg import (
    CanHealth, DrainDetectionArray, MotorStatusArray, TaskState)
from robot_interfaces.srv import AssessDrain, CaptureDrain, ReportDrain


S_BOOT_WAIT = 'BOOT_WAIT'
S_DRIVING = 'DRIVING'
S_SETTLE = 'SETTLE'      # 도착 확정 후 · 진입 전진 전 '가만히 서 있기'
S_MOUNTING = 'MOUNTING'
S_MEASURING = 'MEASURING'
S_ADVANCING = 'ADVANCING'
S_CAPTURING = 'CAPTURING'
S_REPORTING = 'REPORTING'
S_DEPARTING = 'DEPARTING'
S_DONE = 'DONE'
S_PAUSED = 'PAUSED'
S_FAULT = 'FAULT'
S_ABORT = 'ABORT'

#  각 상태에서의 게이트 값.  (allow_drive, servo_enable)
#  ★ 상태를 바꾸면 게이트도 여기서 한 번에 바뀐다 ★
#  게이트를 상태별로 흩어 놓으면 어긋난 조합이 생긴다. 표로 못박아 둔다.
GATES = {
    S_BOOT_WAIT: (False, False),
    S_DRIVING:   (True,  True),
    #  ★ SETTLE 은 '진짜로 선다' ★ allow_drive=False 라 mux 가 STOP 을 낸다.
    #    서보도 재운다 — 안 재우면 ARRIVED 의 0 대신 미세 조정이 계속 나갈 수 있다.
    S_SETTLE:    (False, False),
    S_MOUNTING:  (True,  False),   # 직접 명령하므로 서보는 재운다
    S_MEASURING: (False, False),
    S_ADVANCING: (True,  False),   # 직접 명령하므로 서보는 재운다
    S_CAPTURING: (False, False),
    S_REPORTING: (False, False),
    S_DEPARTING: (True,  False),   # 서보를 끄면 mux 가 회피 전진으로 떨어진다
    S_DONE:      (False, False),
    S_PAUSED:    (False, False),
    S_FAULT:     (False, False),
    S_ABORT:     (False, False),
}

VERDICT_NAME = {0: 'UNKNOWN', 1: 'NOT_NEEDED', 2: 'NEEDED'}


class TaskManagerNode(Node):

    def __init__(self):
        super().__init__('task_manager_node')
        p = self.declare_parameter

        # ── 자동 시작 ────────────────────────────────────────────────────
        #  ★ 기본 false ★ 전원이 들어오면 사람 명령 없이 로봇이 나간다.
        #  이 프로젝트의 안전 규칙은 전부 "사람이 시작한다"를 전제로 한다.
        #  시연 launch 에서만 true 로 켤 것. 벤치에서 실수로 나가지 않게.
        p('autostart_enable', False)
        p('autostart_delay', 10.0)

        # ── 시동 전 점검 ─────────────────────────────────────────────────
        #  캡쳐 순간에 카메라가 없으면 임무 전체가 헛수고가 된다.
        #  못 갈 이유는 출발 전에 찾는 게 싸다.
        #  ★ [2026-08-29] 점검이 실패해도 곧바로 접지 않는다 ★
        #  전원만 넣고 자동으로 도는 구성에서는 **장치가 늦게 준비되는 일이 흔하다.**
        #  실제로 21:26 시험에서 sllidar 가 'Can not start scan: 80008002' 로 늦게 살아나
        #  카운트다운 0초 시점에 /cmd_vel_avoid 가 아직 없었고 **임무가 그대로 ABORT** 됐다.
        #  ABORT 는 되돌아오지 않으므로 **사람이 가서 다시 켜야 한다** — 선 없이 도는 구성에서
        #  가장 흔한 실패가 이것이다. 이 시간 동안 **점검을 반복**하고, 그래도 안 되면 ABORT.
        #  ★ 점검을 무르게 만든 것이 아니다 ★ 통과 조건은 그대로고 기다려 주기만 한다.
        p('preflight_grace', 20.0)
        p('preflight_require_avoid', True)   # 회피 입력(=LiDAR) 살아있는가
        p('preflight_require_can', True)     # 모터 fault 0 · CAN 정상인가

        # ── 도착 판정 ────────────────────────────────────────────────────
        #  ARRIVED 는 area_ratio 기반이라 한 프레임 튀면 오인한다(미해결 16번).
        #  카메라 10Hz 기준 1.0s = 10프레임 연속 유지되어야 인정한다.
        p('arrived_debounce', 1.0)

        # ── 도착 후 정지 (SETTLE) ────────────────────────────────────────
        #  ★ [2026-08-30 사용자 지시] ★ "서보잉 구역 안에 바운딩 박스가 들어가고
        #  전진 120cm 하기 전에 5초간 멈추자"
        #  왜 두는가 — ① 사람이 '여기서 섰다'를 눈으로 확인할 시간을 준다
        #  ② 관성이 남은 채로 개루프 전진에 들어가면 그만큼 더 간다
        #     (정지 명령 후 회전 비트가 꺼지기까지 약 1.0초 · CLAUDE.md 실측)
        #  0 이하면 이 상태를 건너뛴다.
        p('settle_seconds', 5.0)

        # ── 배수구 진입 전진 (MOUNTING) ──────────────────────────────────
        #  ★ 서보의 ARRIVED 는 '배수구 앞'이다 ★ area_ratio 0.25 는 배수구가
        #  화면의 1/4 을 채운 지점일 뿐, 로봇이 그 **위**에 있다는 뜻이 아니다.
        #  TOF 6개가 배수구를 보려면 올라가 있어야 한다. 그래서 n 만큼 더 간다.
        #  ★ mount_distance 는 실측하지 않았다 ★ 기본 0.30 은 근거 없는 출발값이다.
        #  ☆ 정하는 법 ☆ 실제 배수구 앞에서 서보를 세우고, 그 자리에서 배수구
        #     중심까지의 거리를 재서 넣는다. 그 다음 개루프 오차를 mount_calib 로 뺀다.
        p('mount_distance', 0.30)
        p('mount_speed', 0.45)     # 명령 121rpm. 실용 하한 60rpm 위
        p('mount_calib', 1.0)      # 같은 속도라면 advance_calib 과 같은 값이다
        p('mount_grace', 0.5)      # 출발 직후 이 시간은 출처를 안 본다
        #  ★★ [2026-09-06 사용자 지시] 회피가 끼면 시계를 멈추고 기다린다 ★★
        #  원문: "mux source가 TASK일 때만 MOUNTING 시간을 누적 / AVOID가 끼면
        #        timer 일시정지 / 회피 끝나고 TASK가 다시 선택되면 남은 시간부터 계속"
        #  ☆ 왜 예산(budget)이 필요한가 ☆ 무한정 기다리면 임무가 멈춘다.
        #    실측(09-06 로그 8판)에서 회피가 발동한 이유는 **도착 지점의
        #    front 가 0.40~0.49m** 라서다 — safe_distance(0.50) 아래다.
        #    배수구·벽이 바로 앞에 있으니 **회피가 영영 안 물러날 수 있다.**
        #    그러면 mission_timeout(300초)까지 갔다가 ABORT 가 되는데,
        #    그건 지금 동작(그 자리에서 판정)보다 나쁘다.
        #  ☆ 그래서 ☆ 'TASK 가 아닌 채로 보낸 시간'이 이 예산을 넘으면
        #    **옛 동작(그 자리에서 판정)으로 떨어진다.** 2026-08-24 결정이
        #    사라지는 게 아니라 **기다려 본 뒤의 차선책**이 된다.
        #  0 이하로 두면 기다리지 않는다 = 옛 동작으로 정확히 되돌아간다.
        p('mount_wait_budget', 15.0)

        # ── 전진 ─────────────────────────────────────────────────────────
        #  /odom 을 못 쓰므로 개루프(시간 x 속도)다.
        #  저속 손실이 있어(미해결 2번) 명령대로 안 간다 -> advance_calib 로 보정.
        #  ★ 바닥에서 실측해 이 값을 정할 것 ★ (바퀴를 띄우면 의미 없다)
        p('advance_distance', 0.50)
        p('advance_speed', 0.45)      # 명령 121rpm. 실용 하한 60rpm 위
        p('advance_calib', 1.0)       # 실제이동 / 목표이동 의 역수를 넣는다
        p('advance_grace', 0.5)       # 출발 직후 이 시간은 출처를 안 본다
        #  ★ 재시도는 파라미터가 아니라 코드로 못박혀 있다 ★ 재시도 경로가 없다.
        #  (2026-08-24: 안 쓰이던 advance_retry 파라미터를 지웠다.
        #   결정은 아래 _st_advancing 의 주석에 남아 있다)

        #  모터가 실제로 서고 값이 안정되기를 기다리는 시간
        p('settle_time', 0.5)

        # ── 판정 ─────────────────────────────────────────────────────────
        p('assess_sample_count', 0)   # 0 = TOF 노드 기본값
        #  ★ 시험 주입 ★ TOF 노드가 없어도 상태 전이를 검증하려고 둔다.
        #  'OFF' 가 아니면 서비스를 부르지 않고 이 값을 판정으로 쓴다.
        p('test_verdict', 'OFF')      # OFF | NEEDED | NOT_NEEDED | UNKNOWN

        # ── 서비스 ───────────────────────────────────────────────────────
        p('assess_service', '/tof_node/assess_drain')
        p('capture_service', '/rear_camera_node/capture_drain')
        p('report_service', '/comms_node/report_drain')
        #  없는 노드를 기다리는 시간. 지나면 포기하고 진행한다(임무를 안 멈춘다).
        p('service_timeout', 3.0)

        # ── 임무 ─────────────────────────────────────────────────────────
        p('target_count', 1)          # 시연: 배수구 1개 처리하고 종료
        p('mission_timeout', 300.0)
        p('depart_timeout', 5.0)      # target_count >= 2 일 때만 쓰인다
        p('avoid_lost_timeout', 5.0)  # 회피 입력이 이만큼 없으면 ABORT
        p('queue_path', '~/drain_reports.jsonl')
        p('publish_rate', 20.0)

        g = self.get_parameter
        self.autostart = g('autostart_enable').value
        self.autostart_delay = float(g('autostart_delay').value)
        self.preflight_grace = float(g('preflight_grace').value)
        self.pre_avoid = g('preflight_require_avoid').value
        self.pre_can = g('preflight_require_can').value
        self.arrived_debounce = float(g('arrived_debounce').value)
        self.mnt_dist = float(g('mount_distance').value)
        self.settle_sec = float(g('settle_seconds').value)
        self.mnt_speed = float(g('mount_speed').value)
        self.mnt_calib = float(g('mount_calib').value)
        self.mnt_grace = float(g('mount_grace').value)
        self.mnt_wait_budget = float(g('mount_wait_budget').value)
        self.adv_dist = float(g('advance_distance').value)
        self.adv_speed = float(g('advance_speed').value)
        self.adv_calib = float(g('advance_calib').value)
        self.adv_grace = float(g('advance_grace').value)
        self.settle = float(g('settle_time').value)
        self.sample_count = int(g('assess_sample_count').value)
        self.test_verdict = str(g('test_verdict').value).upper()
        self.svc_timeout = float(g('service_timeout').value)
        self.target_count = int(g('target_count').value)
        self.mission_timeout = float(g('mission_timeout').value)
        self.depart_timeout = float(g('depart_timeout').value)
        self.avoid_lost_timeout = float(g('avoid_lost_timeout').value)
        self.queue_path = os.path.expanduser(g('queue_path').value)

        #  전진 시간 = 거리 / (속도 x 보정)
        self.adv_duration = self.adv_dist / max(0.01, self.adv_speed * self.adv_calib)
        #  mount_distance <= 0 이면 진입 전진을 아예 건너뛴다(옛 동작으로 되돌리는 스위치)
        self.mnt_duration = (0.0 if self.mnt_dist <= 0.0 else
                             self.mnt_dist / max(0.01, self.mnt_speed * self.mnt_calib))

        # ── 상태 ─────────────────────────────────────────────────────────
        self._last_pre_why = ''
        self.state = S_BOOT_WAIT
        self.t_state = self._now()
        self.t_mission = self._now()
        self.sequence = 0
        self.detail = ''
        self.started = False
        self._future = None
        self._last_count = -1
        self._arrived_since = None
        self._task_cmd_out = None
        self._advance_failed_reason = ''
        #  ★ MOUNTING 은 '흐른 시간'이 아니라 '실제로 내 명령이 나간 시간'을 센다 ★
        #    _mnt_moved  : mux 출처가 TASK 인 동안만 쌓인다 (= 실제 전진 시간)
        #    _mnt_wait   : 회피가 이기는 동안 쌓인다 (예산 관리용)
        #    _mnt_t      : 직전 틱 시각. 0 이면 이 상태에 막 들어온 것이다.
        self._mnt_moved = 0.0
        self._mnt_wait = 0.0
        self._mnt_t = 0.0
        #  ★ note 와 따로 둔다 ★ _set_verdict 가 self.note 를 덮어쓰기 때문이다.
        #  보고할 때 둘을 합친다(_full_note).
        self.pos_note = ''

        #  판정 결과 보관
        self.verdict = 0
        self.mean_cm = 0.0
        self.valid_cnt = 0
        self.sensors = [0] * 6
        self.image_path = ''
        self.note = ''

        # ── 입력 ─────────────────────────────────────────────────────────
        self.servo_state = ''
        self.mux_src = ''
        self.avoid_t = 0.0
        self.motor_fault = 0
        self.motor_seen = False
        self.can_motor_stale = False
        self.can_seen = False
        self.det_count = 0
        self.det_t = 0.0

        self.create_subscription(String, '/drain_servo_node/state',
                                 self._on_servo_state, 10)
        self.create_subscription(String, '/cmd_mux_node/source',
                                 self._on_mux_src, 10)
        self.create_subscription(Twist, '/cmd_vel_avoid', self._on_avoid, 10)
        self.create_subscription(MotorStatusArray, '/motor/status',
                                 self._on_motor, qos_profile_sensor_data)
        self.create_subscription(CanHealth, '/can/health', self._on_health, 10)
        self.create_subscription(DrainDetectionArray, '/detections/drains',
                                 self._on_det, 10)

        # ── 출력 ─────────────────────────────────────────────────────────
        self.pub_allow = self.create_publisher(Bool, '/task/allow_drive', 10)
        self.pub_servo_en = self.create_publisher(Bool, '/task/servo_enable', 10)
        self.pub_task_cmd = self.create_publisher(Twist, '/cmd_vel_task', 10)
        self.pub_state = self.create_publisher(TaskState, '~/state', 10)

        # ── 서비스 클라이언트 ────────────────────────────────────────────
        self.cli_assess = self.create_client(AssessDrain, g('assess_service').value)
        self.cli_capture = self.create_client(CaptureDrain, g('capture_service').value)
        self.cli_report = self.create_client(ReportDrain, g('report_service').value)

        # ── 사람이 거는 제어 ─────────────────────────────────────────────
        self.create_service(Trigger, '~/start', self._srv_start)
        self.create_service(Trigger, '~/stop', self._srv_stop)

        self.create_timer(1.0 / max(1.0, g('publish_rate').value), self._tick)

        self.get_logger().info(
            f'TaskManager 시작 · autostart={self.autostart}'
            f'(delay={self.autostart_delay:.0f}s) · target={self.target_count}개 · '
            f'진입 {self.mnt_dist:.2f}m -> {self.mnt_duration:.2f}초'
            f'(회피 대기 예산 {self.mnt_wait_budget:.1f}초) · '
            f'전진 {self.adv_dist:.2f}m @ {self.adv_speed:.2f}m/s '
            f'-> {self.adv_duration:.2f}초(보정 {self.adv_calib:.3f})')
        if self.mnt_duration <= 0.0:
            self.get_logger().warn(
                'mount_distance <= 0 — 배수구 진입 전진을 건너뛴다. '
                '서보가 선 자리에서 그대로 TOF 를 읽는다.')
        if self.test_verdict != 'OFF':
            self.get_logger().warn(
                f'test_verdict={self.test_verdict} — TOF 노드를 부르지 않고 '
                '이 값을 판정으로 쓴다. ★시험 전용★')
        if not self.autostart:
            self.get_logger().info(
                'autostart_enable=false — 스스로 출발하지 않는다. '
                '시작하려면:  ros2 service call /task_manager_node/start '
                'std_srvs/srv/Trigger {}')

    # =====================================================================
    #  구독 콜백
    # =====================================================================
    def _on_servo_state(self, m):
        self.servo_state = m.data

    def _on_mux_src(self, m):
        self.mux_src = m.data

    def _on_avoid(self, _m):
        self.avoid_t = self._now()

    def _on_motor(self, m):
        self.motor_seen = True
        worst = 0
        for mt in m.motors:
            if mt.fault_code:
                worst = max(worst, int(mt.fault_code))
        self.motor_fault = worst

    def _on_health(self, m):
        self.can_seen = True
        self.can_motor_stale = bool(m.motor_stale)

    def _on_det(self, m):
        self.det_count = len(m.detections)
        self.det_t = self._now()

    # =====================================================================
    #  사람이 거는 제어
    # =====================================================================
    def _srv_start(self, _req, res):
        if self.state in (S_BOOT_WAIT, S_PAUSED, S_DONE, S_ABORT, S_FAULT):
            self.t_mission = self._now()
            self.sequence = 0
            self._go(S_DRIVING, '사람이 시작')
            res.success, res.message = True, '임무 시작'
        else:
            res.success, res.message = False, f'이미 진행 중: {self.state}'
        return res

    def _srv_stop(self, _req, res):
        self._go(S_PAUSED, '사람이 정지')
        res.success, res.message = True, '정지'
        return res

    # =====================================================================
    #  상태 전이
    # =====================================================================
    def _go(self, state, detail=''):
        if state == self.state:
            return
        self.get_logger().info(f'상태 {self.state} -> {state}'
                               + (f'  ({detail})' if detail else ''))
        self.state = state
        self.t_state = self._now()
        if state == S_MOUNTING:
            #  ★ 이 상태에 들어올 때마다 처음부터 센다 ★ target_count 가 2 이상이면
            #    두 번째 배수구에서 다시 지나므로 지우지 않으면 이어져 버린다.
            self._mnt_moved = 0.0
            self._mnt_wait = 0.0
            self._mnt_t = 0.0
        self.detail = detail
        self._future = None
        self._task_cmd_out = None
        self._arrived_since = None
        self._last_count = -1

    def _elapsed(self):
        return self._now() - self.t_state

    # =====================================================================
    #  본체
    # =====================================================================
    def _tick(self):
        #  1) 어느 상태든 게이트를 먼저 낸다.
        #     상태 처리 중 예외가 나도 게이트는 이미 나가 있어야 한다.
        allow, servo_en = GATES[self.state]
        self.pub_allow.publish(Bool(data=allow))
        self.pub_servo_en.publish(Bool(data=servo_en))
        if self._task_cmd_out is not None:
            self.pub_task_cmd.publish(self._task_cmd_out)

        #  2) 어디서든 걸리는 감시
        self._watchdogs()

        #  3) 상태별 처리
        handler = getattr(self, '_st_' + self.state.lower(), None)
        if handler:
            handler()

        #  4) 이 틱에 상태가 바뀌었으면 새 게이트를 **즉시 한 번 더** 낸다.
        #     ★ 안 그러면 두 가지 문제가 생긴다 ★
        #     ① 새 상태의 게이트가 다음 틱(50ms)까지 늦는다.
        #        MEASURING 처럼 '정지'로 가는 전이에서는 그만큼 늦게 선다.
        #     ② TaskState 의 state 는 새 상태인데 allow/servo_enable 은
        #        직전 상태 값이라 한 틱 동안 서로 어긋난다.
        #        TaskState.msg 에 "상태와 어긋나면 버그다"라고 적어 둔 그 모양이다.
        #        (2026-08-23 첫 시험에서 실제로 이렇게 찍혔다)
        allow2, servo_en2 = GATES[self.state]
        if (allow2, servo_en2) != (allow, servo_en):
            self.pub_allow.publish(Bool(data=allow2))
            self.pub_servo_en.publish(Bool(data=servo_en2))

        #  5) 상태 발행 — 실제로 마지막에 내보낸 게이트 값을 싣는다
        self._publish_state(allow2, servo_en2)

    def _watchdogs(self):
        if self.state in (S_DONE, S_PAUSED, S_FAULT, S_ABORT, S_BOOT_WAIT):
            return
        #  모터 fault -> 즉시 임무 중단. 자동 복귀 없음(사람이 확인해야 한다)
        if self.motor_seen and self.motor_fault:
            self._go(S_FAULT, f'모터 fault_code={self.motor_fault}')
            return
        if self.can_seen and self.can_motor_stale:
            self._go(S_FAULT, 'CAN motor_stale')
            return
        #  회피 입력 영구 두절.  mux 는 0.5s 면 이미 세운다. 여기는 그보다 길게 보고
        #  "왜 서 있는지"를 사람에게 알리는 자리다. /scan 이 조용히 죽은 전력이
        #  있다(미해결 12·12-B). 그때 로봇은 영영 서 있게 된다.
        if (self.avoid_t > 0.0
                and self._now() - self.avoid_t > self.avoid_lost_timeout):
            self._go(S_ABORT, f'회피 입력 {self.avoid_lost_timeout:.0f}초 두절')
            return
        if self._now() - self.t_mission > self.mission_timeout:
            self._go(S_ABORT, f'임무 시간 초과 {self.mission_timeout:.0f}초')

    # ── BOOT_WAIT ────────────────────────────────────────────────────────
    def _st_boot_wait(self):
        if not self.autostart:
            return
        left = self.autostart_delay - self._elapsed()
        n = int(math.ceil(left))
        if n != self._last_count and n >= 0:
            self._last_count = n
            self.detail = f'자동 시작까지 {n}초'
            self.get_logger().warn(f'★ 자동 시작까지 {n}초 ★ (멈추려면 지금)')
        if left > 0.0:
            return
        ok, why = self._preflight()
        if not ok:
            #  ★ 카운트다운이 끝나도 곧바로 접지 않는다 ★ preflight_grace 동안 다시 본다.
            #  장치가 늦게 준비되는 것과 정말로 없는 것은 **기다려 봐야 갈린다.**
            if self._elapsed() < self.autostart_delay + self.preflight_grace:
                waited = self._elapsed() - self.autostart_delay
                self.detail = (f'시동 전 점검 대기 {waited:.0f}/'
                               f'{self.preflight_grace:.0f}초 — {why}')
                if why != self._last_pre_why:
                    self._last_pre_why = why
                    self.get_logger().warn(
                        f'시동 전 점검 미통과: {why} — 최대 '
                        f'{self.preflight_grace:.0f}초 더 기다린다')
                return
            self._go(S_ABORT, f'시동 전 점검 실패: {why} '
                              f'({self.preflight_grace:.0f}초 기다렸다)')
            return
        self.t_mission = self._now()
        self._go(S_DRIVING, '자동 시작')

    def _preflight(self):
        if self.pre_avoid:
            if self.avoid_t == 0.0 or self._now() - self.avoid_t > 1.0:
                return False, '회피 입력 없음(LiDAR/회피 노드 확인)'
        if self.pre_can:
            if not self.motor_seen:
                return False, '/motor/status 없음(can_manager 확인)'
            if self.motor_fault:
                return False, f'모터 fault_code={self.motor_fault}'
            if self.can_seen and self.can_motor_stale:
                return False, 'CAN motor_stale'
        return True, ''

    # ── DRIVING ──────────────────────────────────────────────────────────
    def _st_driving(self):
        #  서보의 ARRIVED 가 연속으로 유지될 때만 도착으로 인정한다.
        if self.servo_state == 'ARRIVED':
            if self._arrived_since is None:
                self._arrived_since = self._now()
            held = self._now() - self._arrived_since
            self.detail = f'도착 확인 중 {held:.1f}/{self.arrived_debounce:.1f}s'
            if held >= self.arrived_debounce:
                self.sequence += 1
                self.pos_note = ''
                self._go(S_SETTLE if self.settle_sec > 0.0 else S_MOUNTING,
                         f'{self.sequence}번째 배수구 도착')
        else:
            #  한 프레임이라도 벗어나면 처음부터 다시 센다
            self._arrived_since = None
            self.detail = f'주행 중 (서보={self.servo_state or "?"})'

    # ── SETTLE ───────────────────────────────────────────────────────────
    #  도착을 확정한 자리에서 settle_seconds 만큼 가만히 선다.
    #  ★ 게이트가 (False, False) 라 명령을 따로 낼 필요가 없다 ★
    #    allow_drive=False 를 본 mux 가 STOP 을 내고, can_manager 는 IDLE 을 보낸다.
    #  ★ 감시(_watchdogs)는 이 상태에서도 돈다 ★ 모터 fault · 회피 입력 두절 ·
    #    임무 시간 초과는 서 있는 동안에도 걸려야 한다.
    def _st_settle(self):
        e = self._elapsed()
        if e >= self.settle_sec:
            self._go(S_MOUNTING, f'{self.settle_sec:.1f}초 정지 완료')
            return
        self.detail = f'도착 후 정지 {e:.1f}/{self.settle_sec:.1f}초'

    # ── MOUNTING ─────────────────────────────────────────────────────────
    #  서보가 세운 자리는 배수구 **앞**이다. 그 위로 n 만큼 더 들어간다.
    #  ★ ADVANCING 과 같은 개루프다 ★ /odom 을 못 쓴다(미해결 15번).
    def _st_mounting(self):
        if self.mnt_duration <= 0.0:
            self._go(S_MEASURING, '진입 전진 없음(mount_distance <= 0)')
            return

        e = self._elapsed()
        now = self._now()

        #  ★★ [2026-09-06] 시계는 '내 명령이 실제로 나가는 동안'만 흐른다 ★★
        #    그전에는 상태에 들어온 뒤 흐른 시간(_elapsed)으로 쟀고, 회피가
        #    끼어들면 **그 자리에서 접었다**(그게 08-24 결정이었다).
        #    실측(09-06 로그): 11판 중 8판이 **0.55초 지점에서 접혔다** —
        #    도착 지점의 front 가 0.40~0.49m 라 회피가 곧바로 이겼기 때문이다.
        #    결과가 "1.20m 를 가야 하는데 거의 안 움직이고 ToF 로 넘어감" 이었다.
        #    이제는 **접지 않고 멈춰서 기다렸다가 남은 시간부터 이어 간다.**
        #  ★ 이 방식은 서보의 RECOVER 와 같은 방식이다 ★
        #    "회피가 이기는 중에도 시간을 세면 아무 데도 안 가고 다 썼다고 착각한다."
        dt = now - self._mnt_t if self._mnt_t > 0.0 else 0.0
        self._mnt_t = now
        #  긴 공백(CPU 경합 등)은 버린다. 안 버리면 안 간 거리를 갔다고 센다.
        if dt < 0.0 or dt > 0.5:
            dt = 0.0

        #  ★ 명령은 기다리는 동안에도 계속 낸다 ★ 멈추면 mux 에서 task_cmd 가
        #    신선하지 않게 되고(③ 탈락), 회피가 순항으로 바뀌는 순간
        #    ⑤ 로 **회피의 탐색 전진**이 통과해 버린다. 그러면 배수구를 지나간다.
        self._task_cmd_out = Twist()
        self._task_cmd_out.linear.x = self.mnt_speed

        #  출발 직후에는 mux 출처가 아직 안 바뀌었을 수 있다 -> 그동안은 '먹힌다'로 친다.
        #  mux 를 아예 못 봤으면(mux 없는 구성) 판단할 근거가 없으므로 역시 '먹힌다'.
        #  ★ grace 를 이동으로 세는 것은 옛 동작과 같다 ★ mount_calib(0.648)을
        #    잰 판이 처음부터 끝까지 TASK 였으므로 보정값이 그대로 유효하다.
        in_control = (e <= self.mnt_grace
                      or not self.mux_src
                      or self.mux_src == 'TASK')

        if in_control:
            if self._mnt_wait > 0.0 and dt > 0.0:
                self.get_logger().info(
                    f'배수구 진입 재개 — 남은 '
                    f'{max(0.0, self.mnt_duration - self._mnt_moved):.2f}초부터 이어 간다 '
                    f'(기다린 시간 {self._mnt_wait:.1f}초)')
                self._mnt_wait = 0.0
            self._mnt_moved += dt
        else:
            self._mnt_wait += dt
            #  ★ 무한정 기다리지 않는다 ★ 도착 지점 앞이 원래 좁으면 회피가
            #    영영 안 물러날 수 있다. 그때는 mission_timeout(300초) ABORT 로
            #    가느니 **옛 동작(그 자리에서 판정)** 이 낫다.
            if self.mnt_wait_budget > 0.0 and self._mnt_wait <= self.mnt_wait_budget:
                self.detail = (f'배수구 진입 대기 {self._mnt_wait:.1f}/'
                               f'{self.mnt_wait_budget:.1f}초 (출처={self.mux_src}) · '
                               f'전진 {self._mnt_moved:.2f}/{self.mnt_duration:.2f}초')
                self.get_logger().info(
                    f'배수구 진입 멈춤 — 회피가 통제권을 쥐고 있다 '
                    f'(출처={self.mux_src} · 대기 {self._mnt_wait:.1f}/'
                    f'{self.mnt_wait_budget:.1f}초 · 전진 {self._mnt_moved:.2f}/'
                    f'{self.mnt_duration:.2f}초)', throttle_duration_sec=2.0)
                return
            #  ★ 여기서도 임무를 접지 않는다 ★ (08-24 결정 그대로)
            #  ADVANCING(50cm) 이 끊기면 잃는 것이 사진 한 장이지만, 여기서 접으면
            #  판정 자체가 없어진다. **덜 정확해도 재는 편이 낫다.**
            #  대신 '제대로 못 올라갔다'를 보고에 실어 서버가 구분할 수 있게 한다.
            self._task_cmd_out = None
            self.pos_note = (f'배수구 진입 중 통제권 상실(출처={self.mux_src}, '
                             f'전진 {self._mnt_moved:.2f}/{self.mnt_duration:.2f}초 · '
                             f'{self._mnt_wait:.1f}초 기다렸다)')
            self.get_logger().warn(self.pos_note + ' — 그 자리에서 판정한다')
            self._go(S_MEASURING, '진입 실패 -> 현 위치에서 판정')
            return

        if self._mnt_moved >= self.mnt_duration:
            self._task_cmd_out = None
            self._go(S_MEASURING,
                     f'{self.mnt_dist:.2f}m 진입 완료(전진 {self._mnt_moved:.2f}초 · '
                     f'상태 체류 {e:.2f}초)')
            return

        self.detail = (f'배수구 진입 {self._mnt_moved:.2f}/{self.mnt_duration:.2f}초')

    # ── MEASURING ────────────────────────────────────────────────────────
    def _st_measuring(self):
        #  모터가 실제로 서고 TOF 값이 안정될 때까지 기다린다
        if self._elapsed() < self.settle:
            self.detail = '정지 안정화 대기'
            return

        #  시험 주입: TOF 노드 없이 상태 전이만 검증할 때
        if self.test_verdict != 'OFF':
            v = {'NEEDED': 2, 'NOT_NEEDED': 1}.get(self.test_verdict, 0)
            self._set_verdict(v, 0.0, 0, [0] * 6, f'주입값 {self.test_verdict}')
            self._after_verdict()
            return

        if self._future is None:
            if not self.cli_assess.service_is_ready():
                if self._elapsed() > self.svc_timeout:
                    #  TOF 노드가 없다. 판정 불가로 두고 그대로 보고한다.
                    self._set_verdict(0, 0.0, 0, [0] * 6, 'TOF 노드 없음')
                    self._after_verdict()
                else:
                    self.detail = 'TOF 노드 대기'
                return
            req = AssessDrain.Request()
            req.sample_count = self.sample_count
            self._future = self.cli_assess.call_async(req)
            self.detail = 'TOF 판정 요청'
            return

        if self._future.done():
            try:
                r = self._future.result()
                self._set_verdict(r.verdict, r.mean_distance_cm,
                                  r.valid_sensor_count, list(r.sensor_distances),
                                  r.detail)
            except Exception as e:
                self._set_verdict(0, 0.0, 0, [0] * 6, f'판정 호출 실패: {e}')
            self._after_verdict()

    def _set_verdict(self, v, mean_cm, valid, sensors, detail):
        self.verdict = int(v)
        self.mean_cm = float(mean_cm)
        self.valid_cnt = int(valid)
        self.sensors = (list(sensors) + [0] * 6)[:6]
        self.note = detail or ''
        self.get_logger().info(
            f'판정 = {VERDICT_NAME.get(self.verdict, "?")} · '
            f'평균 {self.mean_cm:.1f}cm · 유효 {self.valid_cnt}/6 · {self.note}')

    def _after_verdict(self):
        #  ★ 2026-08-24 사용자 결정 — 판정 결과로 갈라지지 않는다 ★
        #  옛 동작은 NEEDED 일 때만 전진·캡쳐했다(38단계 결정).
        #  지금은 **청소가 필요하든 아니든 50cm 전진 후 캡쳐한다.**
        #  "깨끗했다"도 사진이 있어야 서버에서 확인할 수 있기 때문이다.
        #  덤으로 TOF 노드가 없는 지금(판정이 항상 UNKNOWN) 그 경로가 죽지 않는다.
        self._go(S_ADVANCING, f'판정 {VERDICT_NAME.get(self.verdict, "?")} '
                              f'-> 전진 후 캡쳐')

    # ── ADVANCING ────────────────────────────────────────────────────────
    def _st_advancing(self):
        e = self._elapsed()
        self._task_cmd_out = Twist()
        self._task_cmd_out.linear.x = self.adv_speed

        #  출발 직후에는 mux 출처가 아직 안 바뀌었을 수 있다
        if e > self.adv_grace and self.mux_src and self.mux_src != 'TASK':
            #  ★ 거리를 포기한다 ★
            #  회피가 통제권을 가져갔다. 얼마나 갔는지 알 방법이 없으므로
            #  남은 거리를 이어가지 않는다. 캡쳐를 건너뛰고 보고로 간다.
            #  ★ 재시도하지 않는다(사용자 결정 2026-08-23) ★ 재시도하면 배수구가
            #  화면 밖으로 나가 로봇이 탐색을 다시 시작한다. 배수구 1개짜리
            #  시연에서 엉뚱한 데로 돌아다니는 그림이 된다.
            self._task_cmd_out = None
            self._advance_failed_reason = (
                f'전진 중 통제권 상실(출처={self.mux_src}, '
                f'{e:.2f}/{self.adv_duration:.2f}초 지점)')
            self.get_logger().warn(self._advance_failed_reason)
            self.image_path = ''
            self.note = self._advance_failed_reason
            self._go(S_REPORTING, '전진 실패 -> 캡쳐 생략')
            return

        if e >= self.adv_duration:
            self._task_cmd_out = None
            self._go(S_CAPTURING,
                     f'{self.adv_dist:.2f}m 전진 완료(개루프 {e:.2f}초)')
            return

        self.detail = f'전진 {e:.2f}/{self.adv_duration:.2f}초'

    # ── CAPTURING ────────────────────────────────────────────────────────
    def _st_capturing(self):
        if self._elapsed() < self.settle:
            self.detail = '정지 안정화 대기'
            return

        if self._future is None:
            if not self.cli_capture.service_is_ready():
                if self._elapsed() > self.svc_timeout:
                    #  후방 카메라 노드가 아직 없다. 영상 없이 보고한다.
                    self.image_path = ''
                    self.note = '후방 카메라 노드 없음'
                    self.get_logger().warn(self.note + ' — 영상 없이 보고한다')
                    self._go(S_REPORTING, '캡쳐 생략')
                else:
                    self.detail = '후방 카메라 노드 대기'
                return
            req = CaptureDrain.Request()
            req.tag = f'seq{self.sequence:03d}'
            self._future = self.cli_capture.call_async(req)
            self.detail = '캡쳐 요청'
            return

        if self._future.done():
            try:
                r = self._future.result()
                if r.success:
                    self.image_path = r.image_path
                    self.get_logger().info(f'캡쳐 완료: {self.image_path}')
                else:
                    self.image_path = ''
                    self.note = f'캡쳐 실패: {r.detail}'
                    self.get_logger().warn(self.note)
            except Exception as e:
                self.image_path = ''
                self.note = f'캡쳐 호출 실패: {e}'
                self.get_logger().warn(self.note)
            self._go(S_REPORTING)

    # ── REPORTING ────────────────────────────────────────────────────────
    def _st_reporting(self):
        if self._future is None:
            if not self.cli_report.service_is_ready():
                if self._elapsed() > self.svc_timeout:
                    #  통신 노드가 없다. 디스크 큐에만 남기고 진행한다(b안).
                    self._enqueue(sent=False, server_msg='통신 노드 없음')
                    self._next_after_report()
                else:
                    self.detail = '통신 노드 대기'
                return
            self._future = self.cli_report.call_async(self._make_report())
            self.detail = '서버 전송 요청'
            return

        if self._future.done():
            try:
                r = self._future.result()
                self._enqueue(sent=bool(r.success), server_msg=r.server_message)
                if r.success:
                    self.get_logger().info(f'서버 전송 완료: {r.server_message}')
                else:
                    self.get_logger().warn(f'서버 전송 실패: {r.server_message}')
            except Exception as e:
                self._enqueue(sent=False, server_msg=f'호출 실패: {e}')
                self.get_logger().warn(f'서버 전송 호출 실패: {e}')
            self._next_after_report()

    def _make_report(self):
        req = ReportDrain.Request()
        req.sequence = self.sequence
        req.stamp = self.get_clock().now().to_msg()
        req.verdict = self.verdict
        req.mean_distance_cm = self.mean_cm
        req.valid_sensor_count = self.valid_cnt
        req.sensor_distances = [int(x) & 0xFF for x in self.sensors]
        req.image_path = self.image_path
        req.note = self._full_note()
        return req

    def _full_note(self):
        #  판정/캡쳐 쪽 사유(note)와 위치 사유(pos_note)는 서로 다른 실패다.
        #  둘 다 있으면 둘 다 보낸다.
        return ' / '.join(x for x in (self.note, self.pos_note) if x)

    def _enqueue(self, sent, server_msg):
        #  ★ 전송 성공 여부와 무관하게 항상 디스크에 남긴다 ★
        #  통신이 임무를 멈추게 하지 않는다(b안). 나중에 사람이 밀어 넣으면 된다.
        rec = {
            'time': time.strftime('%Y-%m-%dT%H:%M:%S'),
            'sequence': self.sequence,
            'verdict': VERDICT_NAME.get(self.verdict, '?'),
            'mean_distance_cm': round(self.mean_cm, 2),
            'valid_sensor_count': self.valid_cnt,
            'sensor_distances': [int(x) for x in self.sensors],
            'image_path': self.image_path,
            'note': self._full_note(),
            'sent': sent,
            'server_message': server_msg,
        }
        try:
            with open(self.queue_path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(rec, ensure_ascii=False) + '\n')
            self.get_logger().info(f'기록 남김: {self.queue_path}')
        except Exception as e:
            self.get_logger().error(f'기록 실패({self.queue_path}): {e}')

    def _next_after_report(self):
        if self.sequence >= self.target_count:
            self._go(S_DONE, f'{self.sequence}개 처리 완료')
        else:
            self._go(S_DEPARTING, '다음 배수구로')

    # ── DEPARTING ────────────────────────────────────────────────────────
    def _st_departing(self):
        #  서보를 껐으므로 mux 는 회피의 전진으로 떨어진다 = 배수구를 지나간다.
        #  ★ 새 명령 경로를 만들지 않는 것이 요점이다 ★
        #  target_count=1 인 시연에서는 이 상태에 오지 않는다.
        gone = (self._now() - self.det_t > 1.0) or (self.det_count == 0)
        if gone or self._elapsed() > self.depart_timeout:
            self.image_path = ''
            self.note = ''
            self.pos_note = ''
            self._go(S_DRIVING, '이탈 완료')
        else:
            self.detail = f'이탈 중 {self._elapsed():.1f}s'

    # ── 종료 상태들 ──────────────────────────────────────────────────────
    def _st_done(self):
        pass

    def _st_paused(self):
        pass

    def _st_fault(self):
        pass

    def _st_abort(self):
        pass

    # =====================================================================
    def _publish_state(self, allow, servo_en):
        m = TaskState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.state = self.state
        m.sequence = self.sequence
        m.verdict = self.verdict
        m.allow_drive = allow
        m.servo_enable = servo_en
        m.task_cmd_active = self._task_cmd_out is not None
        m.detail = self.detail
        self.pub_state.publish(m)

    def _now(self):
        return time.monotonic()


def main():
    rclpy.init()
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        #  ★ 종료 시 게이트를 '정지'로 내려놓는다 ★
        #  이 노드가 사라지면 mux 는 require_task 에 따라 판단하지만,
        #  마지막 의사를 명시적으로 남기는 편이 낫다.
        try:
            node.pub_allow.publish(Bool(data=False))
            node.pub_servo_en.publish(Bool(data=False))
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
