#!/usr/bin/env python3
# =============================================================================
#  drain_servo_node — 배수로 비주얼 서보잉 (접근 후 정지)
# =============================================================================
#  구독  /detections/drains       robot_interfaces/DrainDetectionArray
#        /cmd_vel                 geometry_msgs/Twist   <- mux 출력. 회전량 적분용
#        /cmd_mux_node/source     std_msgs/String       <- 내 명령이 먹히고 있는가
#  발행  /cmd_vel_servo       geometry_msgs/Twist        <- mux 가 받는다
#        ~/active             std_msgs/Bool              <- mux 가 받는다 (아래 참조)
#        ~/state              std_msgs/String            <- 디버깅용 상태
#
#  ★ '정지하라'와 '의견 없음'은 다르다 ★
#    처음 구현에서 이 둘을 구별하지 않아 **로봇이 배수로를 찾아다닐 수 없었다.**
#    탐지가 없을 때 서보가 0 을 발행했고, mux 는 그것을 신선한 명령으로 보고
#    우선시했다. 그래서 회피 노드의 전진(탐색 주행)이 영영 통과하지 못했다.
#    이제 두 가지를 나눠서 알린다:
#      active=True  : 내가 통제권을 주장한다 (ALIGN/APPROACH/ARRIVED/RECOVER_*/HOLD)
#                     ARRIVED 의 0 은 '여기 서 있어라'라는 **명령**이다
#                     HOLD 의 0 도 명령이다 — 아래 참조
#      active=False : 나는 의견이 없다 (SEARCH). mux 는 회피 쪽을 통과시켜야 한다
#
#  ★★ [2026-09-06] HOLD — 놓쳤다고 해서 가속하면 안 된다 ★★
#    그전에는 탐지가 끊기면 곧바로 SEARCH(active=False)로 떨어졌고, 그러면
#    mux ⑤ 가 **회피의 탐색 전진(0.45 m/s)** 을 그대로 통과시켰다.
#    접근 속도(0.1524)의 **2.95배**다 — 쫓던 배수구를 놓친 그 순간 로봇이
#    느려지는 게 아니라 **3배로 빨라져서 배수구를 지나가 버렸다.**
#    실측(2026-09-06): 도착 로그가 문턱 0.071 인데 **0.085**(20% 초과 = 9% 더
#    가까이)로 찍혔고, 한 세션에서 '놓침 · 복귀 안 함' 4회 / 도착 3회였다.
#    이제 **쫓던 목표를 놓치면 그 자리에 서서**(active=True · 0) 재포착을 기다린다.
#
#  ★★ [2026-09-06] ARRIVED 진입에 가드 둘을 걸었다 ★★
#    증상: LiDAR 회피로 **회전하는 중에** bbox 하단이 화면의 서보잉 견본에
#    살짝 걸친 상태로 ARRIVED 가 걸렸다. TaskManager 는 ~/state 문자열만 보므로
#    (task_manager_node `_st_driving`) 1.0s 뒤 SETTLE -> MOUNTING 으로 넘어가
#    **회피가 돌려놓은 방향으로 개루프 1.2m** 를 달렸다.
#    원인: 도착 조건이 `area_ratio >= stop_area_ratio` **하나**뿐이었고,
#          그 판정이 정렬 판정보다 **먼저** 있었으며, mux 출처를 보지 않았다.
#    수정: 진입에만 ① 정렬됨(ALIGN 이 아님) ② 내 명령이 나가는 중(~/source==SERVO)
#          을 요구한다. **유지(히스테리시스)와 나머지 서보잉은 그대로다.**
#          새 상수는 만들지 않았다 — 기존 align 문턱과 기존 mux 판정을 재사용한다.
#
#  ★ 이 노드는 /cmd_vel 을 직접 쓰지 않는다 ★
#    회피 노드도 cmd_vel 을 쓰기 때문이다. 한 토픽에 발행자가 둘이면
#    누가 이겼는지 알 수 없다. 2026-08-22 에 0x200 을 세 노드가 동시에
#    쓰는 사고를 실제로 냈다. 같은 실수를 토픽 층에서 반복하지 않는다.
#    우선순위 판단은 cmd_mux_node 가 한 곳에서 한다.
#    **읽기는 한다** — 아래 RECOVER 가 /cmd_vel 과 mux 의 ~/source 를 구독한다.
#    읽는 것은 발행자를 늘리지 않고, 우선순위를 다시 판단하지도 않는다.
#
#  ── 왜 '정렬 회전'과 '직진 접근'을 분리했나 (섞어서 조향하지 않는 이유) ──
#    2026-08-22 실측: **명령 rpm 이 60 미만이면 모터가 뚝뚝 끊긴다.**
#    전진하면서 조금씩 조향하면 안쪽 바퀴 명령이 60 아래로 쉽게 내려간다.
#      예) v=0.45, w=0.5 -> 안쪽 (0.45-0.5*0.2)*269 = 94  (아직 괜찮다)
#          v=0.45, w=0.8 -> 안쪽 (0.45-0.8*0.2)*269 = 78
#          v=0.45, w=1.0 -> 안쪽 (0.45-1.0*0.2)*269 = 67  (한계 근처)
#    한계에 붙어서 쓰면 바닥 상태에 따라 쉽게 넘어간다.
#    그래서 **제자리 선회(±121rpm) 아니면 직진(+121rpm)** 둘 중 하나만 낸다.
#    두 동작 모두 명령값이 정확히 121 이라 하한에서 두 배 여유가 있다.
#    회피 노드도 같은 구조다(회전은 linear.x=0 인 제자리 선회).
#
#  ── 히스테리시스 ──
#    정렬 진입/이탈 문턱을 다르게 준다. 같게 주면 문턱 근처에서
#    ALIGN <-> APPROACH 를 오가며 채터링한다.
#    2026-08-21 에 후진 진입/종료에서 똑같은 문제를 겪고 고쳤다.
#
# =============================================================================
#  ★ RECOVER — 회피에 시야를 뺏겨 배수로를 놓쳤을 때 (2026-08-25 사용자 지시) ★
# =============================================================================
#  서보잉 중 장애물이 나타나면 mux 우선순위 ② 때문에 회피가 이긴다. 회피의
#  회전은 **제자리 선회**라서 카메라가 통째로 돌아가고 배수로가 화면 밖으로 나간다.
#
#  그전까지는 그냥 잃었다. 서보는 0.5초 뒤 SEARCH 로 떨어지면서 마지막으로 본
#  방향(last_turn_sign)까지 지웠고, 회피가 끝나면 로봇은 **돌아간 그 방향으로
#  그냥 직진**했다. 되찾는 동작이 어디에도 없었고, 놓쳤다는 사실조차 남지 않았다.
#
#  이제 두 단계로 되찾는다:
#      RECOVER_FWD   recover_distance(0.20m) 만큼 직진
#      RECOVER_TURN  잃는 동안 돌아간 만큼(yaw drift) 반대로 되돌린다
#
#  ★ 왜 되돌리기 전에 20cm 를 먼저 가는가 ★ (사용자 지시)
#    제자리에서 되돌리면 **방금 피한 그 장애물을 다시 정면으로 본다.**
#    회피가 또 이기고(②) 또 돌고, 되돌리고 — 같은 자리에서 맴돈다.
#    20cm 를 먼저 가면 로봇의 위치가 바뀌어 같은 그림이 반복되지 않는다.
#
#  ★ 되돌릴 각도는 /cmd_vel 을 적분해서 얻는다 ★
#    /odom 을 쓸 수 없다(미해결 15번 — 피드백 rpm 이 튄다). 그래서 **mux 가
#    실제로 내보낸 각속도 명령**을 적분해 "얼마나 돌아갔나"를 추정한다.
#    MOUNTING · ADVANCING 과 같은 개루프다. **정확하지 않다** —
#    실물에서 recover_turn_calib 로 맞출 것.
#
#  ★ 목표 방위가 아니라 '돌아간 만큼'만 되돌린다 ★
#    배수로가 화면 끝(cx=0.9)에서 사라졌더라도 사라지기 직전에는 **화면 안에**
#    있었다. 그러니 그 뒤에 돌아간 만큼만 되돌리면 다시 화면에 들어온다.
#    화면 안에서 가운데로 맞추는 일은 ALIGN 이 이어서 한다.
#    cx 를 각도로 바꾸려면 카메라 화각이 필요한데 **실측하지 않았다**(미해결 9번).
#    재지 않은 상수를 코드에 박지 않으려고 이 방식을 골랐다.
#
#  ★ 언제 발동하는가 ★  셋이 모두 참일 때만:
#    ① 쫓고 있던 목표가 있었다 — 그냥 아직 못 찾는 중이면 되찾을 것도 없다
#    ② 잃은 뒤 recover_min_yaw 이상 돌아갔다
#       → **탐지가 깜빡인 것**과 **회피가 시야를 돌린 것**을 가르는 조건이다.
#         깜빡임에 20cm 를 전진하면 ARRIVED 직전에 **배수로를 밟고 지나간다.**
#    ③ 마지막으로 본 지 recover_arm_window 안이다
#       → 그보다 오래 회피했으면 **로봇이 그동안 많이 이동해서**
#         회전만 되돌려도 그 자리에 배수로가 없다.
#
#  ★ 한 번 잃을 때 한 번만 시도한다 ★
#    실패하면 SEARCH 로 떨어지고 **다시 포착했다가 또 잃기 전까지** 재시도하지 않는다.
#    (TaskManager 의 ADVANCING 이 '재시도하지 않는다'와 같은 취지 —
#     헤매느니 탐색 주행으로 돌아가는 편이 낫다)
#
#  ★ 타이머는 '내 명령이 먹히는 동안'만 흐른다 ★
#    회피가 아직 회피 중이면 내 명령은 mux 에서 버려진다. 그때도 시간을 세면
#    **아무 데도 안 가고 20cm 를 다 썼다고 착각한다.** 그래서 mux 의 ~/source 가
#    SERVO 일 때만 시계를 돌린다. (우선순위를 다시 판단하는 게 아니다 —
#    내 명령이 실제로 나가고 있는지 보는 것이다. 판단은 여전히 mux 가 한다)
#    ~/source 를 한 번도 못 받았으면 mux 가 없는 구성으로 보고 '먹힌다'로 친다.
# =============================================================================

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

from robot_interfaces.msg import DrainDetectionArray


# 상태
S_SEARCH = 'SEARCH'      # 탐지 없음 -> 정지 (탐색 주행은 회피 노드가 한다)
S_ALIGN = 'ALIGN'        # 좌우로 틀어져 있음 -> 제자리 선회
S_APPROACH = 'APPROACH'  # 정렬됨, 아직 멀다 -> 직진
S_ARRIVED = 'ARRIVED'    # 충분히 가까움 -> 정지
S_REC_FWD = 'RECOVER_FWD'    # 놓침 복귀 1단계 — 20cm 직진
S_REC_TURN = 'RECOVER_TURN'  # 놓침 복귀 2단계 — 돌아간 만큼 되돌리기
#  ★ [2026-09-06] 쫓던 목표를 놓친 직후 — 그 자리에 서서 재포착을 기다린다 ★
#    SEARCH 와 다른 점은 **통제권을 놓지 않는다**는 것뿐이다(명령은 둘 다 0).
#    놓는 순간 mux ⑤ 로 회피의 탐색 전진 0.45 m/s 가 통과해 배수구를 지나간다.
S_HOLD = 'HOLD'

_REC_STATES = (S_REC_FWD, S_REC_TURN)

#  mux 가 ~/source 로 알려주는 값 중 '서보가 이겼다'를 뜻하는 것.
#  cmd_mux_node.SRC_SERVO 와 같은 문자열이다. 바꾸려면 양쪽을 같이 볼 것.
SRC_SERVO = 'SERVO'


class DrainServoNode(Node):

    def __init__(self):
        super().__init__('drain_servo_node')

        # ── 토픽 ─────────────────────────────────────────────────────────
        self.declare_parameter('detection_topic', '/detections/drains')
        self.declare_parameter('cmd_topic', '/cmd_vel_servo')
        #  RECOVER 전용 입력 둘. 둘 다 **읽기만** 한다.
        self.declare_parameter('cmd_out_topic', '/cmd_vel')
        self.declare_parameter('mux_source_topic', '/cmd_mux_node/source')

        # ── 속도 ─────────────────────────────────────────────────────────
        #  회피 노드 기본값과 일부러 같게 맞췄다. 같은 물리 조건에서 검증된 값이다.
        #  (0.45 m/s = 명령 121rpm · 2.25 rad/s 제자리선회 = 명령 ±121rpm)
        #  ★ 0.223 m/s(=명령 60rpm) 아래로 내리지 말 것. 뚝뚝 끊긴다. ★
        self.declare_parameter('approach_speed', 0.45)
        self.declare_parameter('align_turn_speed', 2.25)

        # ── 정렬 문턱 (cx_norm, -1.0 왼쪽끝 ~ +1.0 오른쪽끝) ──────────────
        #  align_enter 보다 크면 ALIGN 으로 들어가고,
        #  align_exit 보다 작아져야 APPROACH 로 나온다. exit < enter 여야 한다.
        self.declare_parameter('align_enter', 0.20)
        self.declare_parameter('align_exit', 0.10)

        # ── 정지 문턱 (area_ratio = bbox 면적 / 화면 면적) ────────────────
        #  ★ 이것은 거리가 아니라 '거리 대용'이다 ★
        #  같은 거리라도 배수구가 크면 값이 커진다. TOF 가 붙으면 그쪽으로 옮길 것.
        #  stop 보다 커지면 ARRIVED, resume 보다 작아져야 다시 접근한다(히스테리시스).
        #  ★ [2026-09-06] yaml 과 같은 값으로 맞췄다 (0.25/0.18 -> 0.060/0.043) ★
        #    params-file 없이 `ros2 run` 으로 띄우는 경로가 이 값을 쓴다.
        #    옛 0.25 는 화면의 1/4 이라 그 경로에서는 ARRIVED 가 영영 안 걸렸다.
        self.declare_parameter('stop_area_ratio', 0.060)
        self.declare_parameter('resume_area_ratio', 0.043)

        # ── 탐지 선별 ────────────────────────────────────────────────────
        self.declare_parameter('min_score', 0.40)
        #  탐지가 이 시간 넘게 없으면 SEARCH(또는 RECOVER)로 떨어진다.
        #  카메라가 10Hz 이므로 0.5s 는 프레임 5개다. 한두 개 놓쳐도 안 흔들린다.
        self.declare_parameter('detection_timeout', 0.5)

        # ── 놓침 복귀 (RECOVER) ──────────────────────────────────────────
        #  ★ 아래 다섯 개는 하나도 실측하지 않았다 ★ 실물 앞에서 맞출 것.
        self.declare_parameter('recover_enable', True)
        #  되돌리기 전에 앞으로 나가는 거리. 같은 자리에서 맴도는 것을 막는다.
        self.declare_parameter('recover_distance', 0.20)
        #  개루프 전진 보정. TaskManager 의 advance_calib 와 같은 뜻이다.
        #  실제 이동거리 D 를 재서 recover_calib = recover_distance / D.
        self.declare_parameter('recover_calib', 1.0)
        #  이만큼(rad) 이상 돌아갔을 때만 '회피가 시야를 돌렸다'로 본다.
        #  0.30rad = 약 17도. 카메라 화각을 재지 않았으므로(미해결 9번)
        #  이 값은 **깜빡임과 회전을 가르는 출발점일 뿐**이다.
        self.declare_parameter('recover_min_yaw', 0.30)
        #  되돌리기 회전의 개루프 보정.
        self.declare_parameter('recover_turn_calib', 1.0)
        #  되돌리기 회전의 상한(초). 오래 돈 만큼 그대로 되돌리면 한참 돈다.
        self.declare_parameter('recover_turn_max', 3.0)
        #  마지막 탐지로부터 이 시간 안에서만 복귀를 시도한다.
        #  통제권을 못 받고 기다리는 예산으로도 같은 값을 쓴다.
        self.declare_parameter('recover_arm_window', 5.0)

        # ── 놓침 유지 (HOLD) ────────────────────────────────────────────
        #  ★ [2026-09-06 사용자 결정] 놓쳤을 때 가속 금지 ★
        #  쫓던 목표를 놓친 뒤 이 시간 동안은 통제권을 쥔 채 **그 자리에 선다**.
        #  0 이하면 이 동작을 끈다(= 놓치는 즉시 SEARCH · 옛 동작).
        #  ★ 기본값을 recover_arm_window 와 같은 5.0 으로 둔 것은 의도다 ★
        #    복귀(RECOVER)가 발동할지 지켜보는 창과 같은 창이라, 그 창이 닫힐 때
        #    둘이 함께 끝나 '서 있다가 갑자기 달린다'는 틈이 안 생긴다.
        self.declare_parameter('lost_hold_seconds', 5.0)

        # ── 발행 주기 ────────────────────────────────────────────────────
        #  탐지 콜백이 아니라 타이머로 발행한다.
        #  탐지가 끊겨도 **정지 명령을 계속 내보내야** 하기 때문이다.
        #  (발행을 멈추면 mux 와 can_manager 의 타임아웃에 기대게 되는데,
        #   기대는 것보다 명시적으로 0 을 보내는 편이 낫다)
        self.declare_parameter('publish_rate', 20.0)

        p = self.get_parameter
        self.approach_speed = p('approach_speed').value
        self.align_turn_speed = p('align_turn_speed').value
        self.align_enter = p('align_enter').value
        self.align_exit = p('align_exit').value
        self.stop_area = p('stop_area_ratio').value
        self.resume_area = p('resume_area_ratio').value
        self.min_score = p('min_score').value
        self.det_timeout = p('detection_timeout').value

        self.rec_enable = bool(p('recover_enable').value)
        self.rec_distance = float(p('recover_distance').value)
        self.rec_calib = float(p('recover_calib').value)
        self.rec_min_yaw = float(p('recover_min_yaw').value)
        self.rec_turn_calib = float(p('recover_turn_calib').value)
        self.rec_turn_max = float(p('recover_turn_max').value)
        self.rec_window = float(p('recover_arm_window').value)
        self.lost_hold = float(p('lost_hold_seconds').value)

        if self.align_exit >= self.align_enter:
            self.get_logger().warn(
                f'align_exit({self.align_exit}) >= align_enter({self.align_enter}) — '
                '히스테리시스가 없다. 문턱 근처에서 채터링한다.')
        if self.resume_area >= self.stop_area:
            self.get_logger().warn(
                f'resume_area_ratio({self.resume_area}) >= stop_area_ratio({self.stop_area}) — '
                '히스테리시스가 없다. 정지/재출발을 반복한다.')

        # ── 상태 ─────────────────────────────────────────────────────────
        self.state = S_SEARCH
        self.last_det = None        # (cx_norm, area_ratio, score)
        self.last_det_time = 0.0
        self.last_turn_sign = 0.0   # ALIGN 중 회전 방향 유지용

        # ── RECOVER 상태 ─────────────────────────────────────────────────
        self.yaw = 0.0              # /cmd_vel 각속도 적분값 [rad] (절대 방위 아님)
        self._yaw_t = 0.0
        self.yaw_at_det = 0.0       # 마지막으로 배수로를 본 순간의 yaw
        self.had_target = False     # 쫓고 있던 목표가 있었나
        self.rec_used = False       # 이번 상실에 대해 이미 시도했나
        self.rec_t0 = 0.0           # 복귀 진입 시각 (통제권 대기 예산)
        self.rec_left = 0.0         # 현재 단계 남은 시간 [s]
        self.rec_turn_sign = 0.0
        self.rec_turn_secs = 0.0
        self.mux_src = ''
        self.mux_seen = False

        self.cmd_pub = self.create_publisher(Twist, p('cmd_topic').value, 10)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        #  통제권 주장 여부. mux 가 이것을 보고 서보를 쓸지 정한다.
        self.active_pub = self.create_publisher(Bool, '~/active', 10)
        self.create_subscription(
            DrainDetectionArray, p('detection_topic').value, self._on_detections, 10)
        self.create_subscription(
            Twist, p('cmd_out_topic').value, self._on_cmd_out, 10)
        self.create_subscription(
            String, p('mux_source_topic').value, self._on_mux_source, 10)

        self.tick_period = 1.0 / max(1.0, p('publish_rate').value)
        self.create_timer(self.tick_period, self._tick)

        self.get_logger().info(
            f"DrainServo 시작 · {p('detection_topic').value} -> {p('cmd_topic').value} · "
            f'접근={self.approach_speed}m/s 선회={self.align_turn_speed}rad/s · '
            f'정렬 {self.align_enter}/{self.align_exit} · 정지 면적비 {self.stop_area}')
        if self.rec_enable:
            self.get_logger().info(
                f'놓침 복귀 켜짐 · 전진 {self.rec_distance}m -> 돌아간 만큼 되돌리기 '
                f'(발동 {self.rec_min_yaw}rad 이상 · 창 {self.rec_window}s · '
                f'회전 상한 {self.rec_turn_max}s)')
        else:
            self.get_logger().warn('놓침 복귀 꺼짐(recover_enable=false) — 놓치면 그대로 잃는다')

    # =====================================================================
    def _on_detections(self, msg: DrainDetectionArray):
        """가장 그럴듯한 배수로 하나를 고른다.

        고르는 기준은 **면적이 가장 큰 것**이다. 점수가 아니다.
        점수가 높아도 멀리 있는 것을 쫓으면 가까운 것을 지나친다.
        min_score 로 먼저 거른 뒤 면적으로 고른다.
        """
        best = None
        for d in msg.detections:
            if d.score < self.min_score:
                continue
            if best is None or d.area_ratio > best.area_ratio:
                best = d

        if best is None:
            return          # 탐지 없음. last_det_time 을 갱신하지 않아 곧 타임아웃된다.

        self.last_det = (float(best.cx_norm), float(best.area_ratio), float(best.score))
        self.last_det_time = self._now()
        #  ★ 이 순간의 방위를 붙잡아 둔다 ★ 나중에 '그 뒤로 얼마나 돌아갔나'를
        #    재는 기준점이다. 여기서 안 잡으면 되돌릴 각도를 알 수 없다.
        self.yaw_at_det = self.yaw

    def _on_cmd_out(self, msg: Twist):
        """mux 가 실제로 내보낸 명령을 적분해 '얼마나 돌아갔나'를 추정한다.

        ★ 절대 방위가 아니다 ★ 기준점이 없는 누적값이고, 쓰는 것은 두 시점의
        **차이**뿐이다. 명령을 적분한 것이라 실제 회전과 다르다 — /odom 을
        믿을 수 없어서(미해결 15번) 이게 지금 있는 것 중 가장 나은 추정이다.
        """
        now = self._now()
        if self._yaw_t > 0.0:
            dt = now - self._yaw_t
            #  긴 공백은 버린다. mux 가 잠깐 멈췄다 돌아온 것을 회전으로 세면 안 된다.
            if 0.0 < dt < 0.5:
                self.yaw += float(msg.angular.z) * dt
        self._yaw_t = now

    def _on_mux_source(self, msg: String):
        self.mux_src = msg.data
        self.mux_seen = True

    # =====================================================================
    def _tick(self):
        """상태를 갱신하고 명령을 낸다. 발행은 여기서만 한다."""
        now = self._now()
        fresh = (self.last_det is not None
                 and (now - self.last_det_time) <= self.det_timeout)

        cmd = Twist()

        if not fresh:
            self._tick_lost(now)
            return

        # ── 탐지가 살아 있다 ─────────────────────────────────────────────
        if self.state in _REC_STATES:
            #  복귀가 통했다. 여기서 굳이 남은 회전을 마저 돌 이유가 없다.
            self.get_logger().info('복귀 중 배수로 재포착 — 서보잉으로 돌아간다')
            self.state = S_SEARCH       # 아래 판정이 곧바로 다시 정한다
        #  목표를 다시 손에 쥐었으니 다음 상실 때 복귀를 한 번 더 쓸 수 있다.
        self.rec_used = False

        cx, area, _score = self.last_det

        # ── 이미 도착해 서 있는 중인가 (히스테리시스) ──
        #  ★ 유지는 예전 그대로다 ★ 한 번 확정한 도착을 회피가 끼어들었다고
        #    풀지 않는다. 풀면 TaskManager 의 arrived_debounce 가 계속 리셋돼
        #    임무가 SETTLE 로 못 넘어간다. 아래에서 조인 것은 **진입**뿐이다.
        if self.state == S_ARRIVED:
            #  멀어져야 다시 움직인다 (히스테리시스)
            if area < self.resume_area:
                self.state = S_APPROACH
            else:
                self.had_target = True
                self._publish(cmd)      # 계속 정지
                return

        # ── 정렬 상태 판정 (히스테리시스) ──
        #  ★★ [2026-09-06] 도착 판정보다 **먼저** 한다 (순서가 바뀌었다) ★★
        #    그전에는 도착이 먼저라 **정렬을 한 번도 안 보고** ARRIVED 가 걸렸다.
        #    배수구가 화면 가장자리에 있어도 면적비만 넘으면 그 자리에서 도착이었다.
        #    순서를 바꿨으므로 '도착이 막히면 반드시 ALIGN 이 돌아 고친다' 가 성립한다
        #    (문턱 사이 불감대에 갇혀 도착도 정렬도 못 하는 구멍이 안 생긴다).
        if self.state == S_ALIGN:
            if abs(cx) <= self.align_exit:
                self.state = S_APPROACH
                self.last_turn_sign = 0.0
        else:
            if abs(cx) >= self.align_enter:
                self.state = S_ALIGN

        # ── 도착 판정 ────────────────────────────────────────────────────
        #  ★★ [2026-09-06] 면적비 하나로 도착을 확정하지 않는다 ★★
        #    증상: LiDAR 회피로 **회전하는 중에** bbox 가 잠깐 커져 ARRIVED 가 걸렸고,
        #    TaskManager 가 그 문자열만 보고(task_manager_node `_st_driving`)
        #    1.0s 뒤 SETTLE -> MOUNTING 으로 넘어가 **회피가 돌려놓은 엉뚱한 방향으로
        #    개루프 1.2m 를 달렸다.**
        #    ☆ 왜 면적비만으로는 부족한가 ☆ area_ratio 는 bbox **면적** 하나이고
        #      화면 어디에 있는지·어떻게 겹치는지를 담지 않는다(08-24 규약대로
        #      화면의 주황 견본은 넓이 견본일 뿐 '와야 할 자리'가 아니다).
        #      비스듬히 보면 같은 거리에서도 bbox 가 늘어나 값이 부푼다.
        #    ★ 그렇다고 '주황 안에 완전히 들어와야 한다'로 만들면 안 된다 ★
        #      09-06 실측에서 **정상 접근 때도 bbox 윗변이 51px 삐져나온다**
        #      (배수구는 cy_norm 0.50 인데 견본은 화면 아래끝에 그린다).
        #      포함을 요구하면 ARRIVED 가 영영 안 걸려 배수구를 밟고 지나간다.
        #    ☆ 그래서 새 상수를 만들지 않고 **이미 있는 판정 둘**만 진입에 걸었다 ☆
        #      ① 정렬됐을 때만 (align_enter/exit 히스테리시스 그대로)
        #      ② 내 명령이 실제로 나가는 중일 때만 (mux ~/source == SERVO)
        if area >= self.stop_area:
            if self.state == S_ALIGN:
                #  아직 틀어져 있다. 아래 ALIGN 분기가 제자리 선회로 가운데를 맞춘다.
                #  맞춰지면 다음 틱에 곧바로 ARRIVED 가 걸린다.
                self.get_logger().info(
                    f'도착 보류 — 면적비 {area:.3f} 는 넘었으나 정렬 전이다 '
                    f'(cx {cx:+.2f}, 정렬 문턱 {self.align_exit})',
                    throttle_duration_sec=2.0)
            elif not self._in_control():
                #  회피가 통제권을 쥐고 있다 = 지금 로봇을 모는 것은 내가 아니다.
                #  이 자세는 내가 만든 자세가 아니므로 도착으로 확정하지 않는다.
                #  ★ 여기서 0 을 내는 것은 안전한 쪽이다 ★ 이미 '설 만큼 가깝다'는
                #    것은 아는 상태다. 회피가 손을 떼는 순간 전진 명령이 한 틱이라도
                #    나가면 그만큼 더 다가간다 — 그 한 틱을 안 주려고 0 을 낸다.
                self.get_logger().info(
                    f'도착 보류 — 면적비 {area:.3f} 는 넘었으나 회피가 통제권을 쥐고 있다 '
                    f'(출처={self.mux_src or "?"})', throttle_duration_sec=2.0)
                #  ★★ 여기서 SEARCH 로 남으면 안 된다 ★★
                #    _publish 는 active = (state != SEARCH) 로 낸다. SEARCH 에서 곧바로
                #    '가깝고 정렬됨'으로 들어오면(17:48 로그의 실제 경로다) 이 가지에서
                #    **0 을 내면서 active=False** 가 나가고, 그러면 mux ⑤ 가 회피의
                #    **탐색 전진 0.45 m/s** 를 통과시킨다. 서보는 영영 통제권을 못 받아
                #    **도착도 못 걸고 배수구를 지나간다**(가드가 만든 교착).
                #    그래서 APPROACH 로 둔다 — 명령은 0 이지만 **통제권은 쥔다.**
                #    회피가 손을 떼면 mux 가 SERVO 로 넘기고 다음 틱에 ARRIVED 가 걸린다.
                self.state = S_APPROACH
                self.last_turn_sign = 0.0
                self.had_target = True
                self._publish(cmd)      # 0 — '여기 서 있어라'. active=True 다
                return
            else:
                self.state = S_ARRIVED
                self.last_turn_sign = 0.0
                self.had_target = True
                self.get_logger().info(
                    f'도착 — 면적비 {area:.3f} >= {self.stop_area} · 정렬 {cx:+.2f} · 정지',
                    throttle_duration_sec=2.0)
                self._publish(cmd)
                return

        if self.state == S_ALIGN:
            #  cx > 0 = 배수로가 화면 오른쪽 -> 오른쪽으로 돌아야 한다.
            #  ROS 관례상 angular.z 는 +가 반시계(좌회전)이므로 부호를 뒤집는다.
            sign = -1.0 if cx > 0 else 1.0
            #  회전 중 방향을 유지한다. cx 가 0 근처에서 부호만 떨면
            #  좌우로 덜덜 떠는 것을 막는다. (회피 노드의 direction_hold 와 같은 취지)
            if self.last_turn_sign != 0.0 and abs(cx) < self.align_enter:
                sign = self.last_turn_sign
            self.last_turn_sign = sign
            cmd.linear.x = 0.0
            cmd.angular.z = sign * self.align_turn_speed
        else:
            self.state = S_APPROACH
            cmd.linear.x = self.approach_speed
            cmd.angular.z = 0.0

        self.had_target = True
        self._publish(cmd)

    # =====================================================================
    #  탐지가 없을 때. 복귀할 것인가, 그냥 놓을 것인가.
    # =====================================================================
    def _tick_lost(self, now: float):
        if self.state in _REC_STATES:
            self._run_recover(now)
            return
        if self._try_arm_recover(now):
            self._run_recover(now)
            return
        #  ★★ [2026-09-06] 복귀는 안 하지만, 아직 놓으면 안 된다 ★★
        #    쫓던 목표가 있었고 놓친 지 얼마 안 됐으면 **통제권을 쥔 채 선다**.
        #    여기서 통제권을 놓으면 mux ⑤ 가 회피의 탐색 전진(0.45 m/s)을
        #    통과시키는데, 그건 접근 속도(0.1524)의 2.95배다 —
        #    **놓친 그 순간 로봇이 3배로 빨라져 배수구를 지나간다.**
        #    ★ 이것이 회피를 이기는 것은 아니다 ★ mux 우선순위에서 서보(④)는
        #      '회피가 회피 중'(②)보다 아래다. 장애물이 있으면 회피가 그대로 이긴다.
        #      바뀌는 것은 **회피가 그냥 순항 중일 때 그 순항을 통과시키지 않는 것**뿐이다.
        gone = now - self.last_det_time
        if self.lost_hold > 0.0 and self.had_target and gone <= self.lost_hold:
            if self.state != S_HOLD:
                self.get_logger().info(
                    f'배수로 놓침 — 그 자리에 서서 기다린다 '
                    f'(최대 {self.lost_hold:.1f}s · 탐색 전진으로 넘기지 않는다)')
            self.state = S_HOLD
            self.last_turn_sign = 0.0
            self._publish(Twist())
            return

        #  여기까지 왔으면 정말 놓은 것이다 -> 정지하고 '의견 없음'을 알린다.
        #   탐색 주행은 이 노드가 하지 않는다. 회피 노드가 전진하고 있으면
        #   mux 가 그쪽을 통과시킨다. 여기서 굳이 움직이면 두 의도가 겹친다.
        if self.state == S_HOLD:
            self.get_logger().warn(
                f'{self.lost_hold:.1f}s 기다렸지만 배수로가 안 돌아왔다 — '
                f'통제권을 놓는다. 탐색 주행으로 돌아간다')
        self.state = S_SEARCH
        self.last_turn_sign = 0.0
        self._publish(Twist())

    def _try_arm_recover(self, now: float) -> bool:
        """복귀를 시작할 조건인지 본다. 시작하면 상태를 RECOVER_FWD 로 옮긴다."""
        if not self.rec_enable or self.rec_used or not self.had_target:
            return False

        gone = now - self.last_det_time
        if gone > self.rec_window:
            #  너무 오래 지났다. 그동안 로봇이 이동했을 테니 회전만 되돌려도 소용없다.
            #  ★ 여기서 rec_used 를 세워 이번 상실에 대한 시도를 닫는다 ★
            self.rec_used = True
            self.had_target = False
            self.get_logger().warn(
                f'배수로 놓침 · 복귀 안 함 — 마지막 탐지로부터 {gone:.1f}s '
                f'> {self.rec_window:.1f}s (그동안 이동했다). 탐색 주행으로 돌아간다')
            return False

        drift = self.yaw - self.yaw_at_det
        if abs(drift) < self.rec_min_yaw:
            #  아직 판단하지 않는다. 회피가 더 돌면 그때 발동한다.
            #  ★ 여기서 포기하면 안 된다 ★ 잃은 직후에는 회피가 이제 막 돌기
            #    시작한 참이라 drift 가 작다. 창(rec_window)이 닫힐 때까지 지켜본다.
            return False

        #  ★ 되돌릴 각도는 여기서 정하지 않는다 ★
        #    지금은 회피가 아직 돌고 있는 중일 수 있다. 여기서 굳어 버리면
        #    **그 뒤에 더 돈 만큼을 못 되돌린다.** 전진을 마치고 실제로
        #    되돌리기 직전에 그때의 drift 로 다시 계산한다(_run_recover).
        self.rec_turn_sign = 0.0
        self.rec_turn_secs = 0.0
        self.rec_left = (self.rec_distance * self.rec_calib
                         / max(0.05, self.approach_speed))
        self.state = S_REC_FWD
        self.rec_t0 = now
        self.rec_used = True
        self.get_logger().warn(
            f'★ 배수로 놓침 · 복귀 시작 ★ 여기까지 회피가 {drift:+.2f}rad 돌렸다 · '
            f'먼저 {self.rec_distance:.2f}m 전진({self.rec_left:.2f}s), '
            f'되돌릴 각도는 전진을 마치고 그때 값으로 정한다')
        return True

    def _run_recover(self, now: float):
        """복귀 동작을 한 틱 진행한다. 시계는 내 명령이 먹힐 때만 흐른다."""
        in_control = self._in_control()

        cmd = Twist()
        if self.state == S_REC_FWD:
            cmd.linear.x = self.approach_speed
        else:
            cmd.angular.z = self.rec_turn_sign * self.align_turn_speed
        self._publish(cmd)

        if not in_control:
            #  회피가 아직 통제권을 쥐고 있다. 시계를 멈추고 기다린다.
            #  ★ 여기서 세면 아무 데도 안 가고 20cm 를 다 썼다고 착각한다 ★
            waited = now - self.rec_t0
            if waited > self.rec_window:
                self.get_logger().warn(
                    f'복귀 포기 — {waited:.1f}s 동안 통제권을 못 받았다 '
                    f'(회피가 계속 이기는 중, 출처={self.mux_src or "?"})')
                self._abandon_recover()
            else:
                self._log_waiting(waited)
            return

        self.rec_left -= self.tick_period
        if self.rec_left > 0.0:
            return

        if self.state == S_REC_FWD:
            #  ★ 되돌릴 각도를 지금 정한다 ★ 전진하는 동안(그리고 통제권을
            #    기다리는 동안) 회피가 더 돌았을 수 있다. 발동 시점의 값으로
            #    굳혀 두면 그 몫을 못 되돌린다. 전진은 직진이라 여기에 각을 더하지 않는다.
            drift = self.yaw - self.yaw_at_det
            secs = min(abs(drift) * self.rec_turn_calib
                       / max(0.1, self.align_turn_speed), self.rec_turn_max)
            if secs < self.tick_period:
                #  되돌릴 것이 없다(회피가 돌았다가 도로 돌아왔거나 보정이 0).
                self.get_logger().info(
                    f'복귀 전진 완료 · 되돌릴 각도 없음({drift:+.2f}rad) — 탐색 주행으로')
                self._abandon_recover()
                return
            self.rec_turn_sign = -1.0 if drift > 0.0 else 1.0
            self.rec_turn_secs = secs
            self.state = S_REC_TURN
            self.rec_left = secs
            self.rec_t0 = now
            self.get_logger().info(
                f'복귀 전진 완료 · 총 {drift:+.2f}rad 돌아가 있다 — '
                f'{"좌" if self.rec_turn_sign > 0 else "우"}로 {secs:.2f}s 되돌린다'
                + (' [상한에 걸림]' if secs >= self.rec_turn_max else ''))
        else:
            self.get_logger().warn(
                '복귀 회전 끝 — 배수로를 다시 못 찾았다. 탐색 주행으로 돌아간다')
            self._abandon_recover()

    def _abandon_recover(self):
        self.state = S_SEARCH
        self.last_turn_sign = 0.0
        self.had_target = False
        self.rec_left = 0.0

    def _in_control(self) -> bool:
        """지금 mux 가 내 명령을 내보내고 있는가.

        ★ 우선순위를 다시 판단하는 게 아니다 ★ 판단은 여전히 cmd_mux_node
        한 곳에서 한다. 여기서는 그 결과를 **읽기만** 한다.
        mux 를 한 번도 못 봤으면 mux 없는 구성(벤치)으로 보고 '먹힌다'로 친다.
        """
        return (not self.mux_seen) or (self.mux_src == SRC_SERVO)

    def _log_waiting(self, waited: float):
        self.get_logger().info(
            f'복귀 대기 중 — 회피가 통제권을 쥐고 있다 '
            f'({waited:.1f}/{self.rec_window:.1f}s)', throttle_duration_sec=1.0)

    # =====================================================================
    def _publish(self, cmd: Twist):
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(String(data=self.state))
        #  SEARCH 만 '의견 없음'이다. ARRIVED 의 0 은 명령이므로 active=True.
        #  RECOVER_* 도 명령이다 — 내가 몰아야 20cm 를 가고 되돌릴 수 있다.
        #  ★ HOLD 의 0 도 명령이다 ★ '놓쳤으니 여기 서서 기다려라'는 뜻이고,
        #    여기서 active=False 를 내면 회피의 탐색 전진이 통과해 지나가 버린다.
        self.active_pub.publish(Bool(data=(self.state != S_SEARCH)))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main():
    rclpy.init()
    node = DrainServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        #  종료 시 정지 명령을 한 번 내보낸다.
        #  (mux 가 서보 명령을 붙들고 있지 않도록)
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
