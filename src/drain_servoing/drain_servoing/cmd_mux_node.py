#!/usr/bin/env python3
# =============================================================================
#  cmd_mux_node — /cmd_vel 조정자. 이 노드만 /cmd_vel 을 발행한다.
# =============================================================================
#  구독  /cmd_vel_avoid   geometry_msgs/Twist   <- obstacle_avoidance_node (리맵)
#        /cmd_vel_servo   geometry_msgs/Twist   <- drain_servo_node
#  발행  /cmd_vel         geometry_msgs/Twist   -> can_manager
#        ~/source         std_msgs/String       <- 지금 누가 이겼는지 (디버깅)
#
#  ★ 존재 이유 ★
#    2026-08-22 에 can_manager 노드 3개가 동시에 0x200 을 쏘는 사고를 냈다.
#    프레임 간격이 17/43/40ms 로 찍히기 전까지 아무도 몰랐다.
#    **한 채널에 발행자가 둘이면 누가 이겼는지 알 수 없다.**
#    토픽 층에서 같은 실수를 반복하지 않으려고 조정을 한 곳에 모았다.
#    회피 노드는 리맵으로 붙인다 — 코드를 고치지 않는다.
#      ros2 run drain_lidar_avoidance obstacle_avoidance_node \
#          --ros-args -r /cmd_vel:=/cmd_vel_avoid
#
#  구독(추가)  /task/allow_drive    std_msgs/Bool   <- TaskManager (임무 게이트)
#              /task/servo_enable   std_msgs/Bool   <- TaskManager (서보 게이트)
#              /cmd_vel_task        Twist           <- TaskManager (정밀 동작)
#
#  ── 우선순위 ──
#    ⓪ 임무 게이트가 '정지'다      -> 정지.
#       TaskManager 가 MEASURING/CAPTURING/REPORTING/DONE 에서 세우는 자리다.
#       ★ 이건 '임무 정지'이지 '안전 정지'가 아니다 ★ — 안전은 여전히
#       can_manager 의 cmd_vel 0.3s / motor_status 1.2s / ECU 워치독이 담당한다.
#    ① 회피 입력이 끊겼다        -> 정지.  ★가장 중요★
#       회피 노드가 죽었는데 서보가 계속 달리면 장애물 감시 없이 주행하게 된다.
#       /scan 이 3초씩 조용히 끊긴 전력이 있다(미해결 12번). 그때도 여기서 선다.
#    ② 회피가 '회피 중'이다      -> 회피가 이긴다.
#       ★ TASK 보다 위에 둔 것은 의도다 ★ — 임무 목표(50cm 전진)와 안전이
#       충돌하면 안전이 이긴다. 그 대신 전진 거리는 포기한다.
#       TaskManager 가 출처를 감시하다가 TASK 를 뺏기면 전진을 실패 처리한다.
#    ③ TaskManager 가 직접 명령 중 -> TASK 가 이긴다.
#       게이트만으로는 '50cm 전진' 같은 정밀 동작을 만들 수 없어서 둔 경로다.
#    ④ 회피가 '전진 중'(길이 열림) -> **서보가 통제권을 주장할 때만** 서보가 이긴다.
#       주장하지 않으면(탐지 없음) 회피의 전진을 그대로 통과시킨다 = 탐색 주행.
#
#  ── 게이트가 없을 때(TaskManager 미기동) ──
#    두 게이트 모두 **허용으로 간주**한다. TaskManager 없이도 오늘까지의 구성이
#    그대로 돌아야 하기 때문이다(서보 단독 시험 · 벤치 시험).
#    ★ 바닥에서는 require_task: true 를 줄 것 ★ — 그러면 게이트가 없으면 정지한다.
#
#  ★ 서보의 '0' 을 그냥 믿으면 안 된다 ★
#    처음 구현은 서보가 신선하기만 하면 우선시했다. 그런데 서보는 탐지가 없을 때도
#    0 을 발행한다. 그래서 **로봇이 배수로를 찾아 돌아다닐 수 없었다**(시나리오 ②로 잡음).
#    ARRIVED 의 0('여기 서 있어라')과 SEARCH 의 0('의견 없음')은 다른 뜻이다.
#    서보가 ~/active 로 그 구분을 알려준다.
#
#  ── '회피 중'을 어떻게 아는가 ──
#    회피 노드는 상태를 발행하지 않는다. 그래서 Twist 모양으로 판별한다.
#    obstacle_avoidance_node 의 세 분기가 서로 다른 모양을 낸다:
#        후진   linear.x < 0,  angular.z = 0
#        회전   linear.x = 0,  angular.z = ±turn_speed   (제자리 선회)
#        전진   linear.x > 0,  angular.z = 0
#    따라서 **linear.x > 0 이고 angular.z ≈ 0 일 때만 '길이 열렸다'**로 본다.
#    ⚠ 이 판별은 회피 노드의 분기 모양에 의존한다.
#       회피 노드가 '전진하면서 조향'하도록 바뀌면 여기도 같이 고쳐야 한다.
#       (그때는 회피 노드가 상태 토픽을 내보내게 하는 편이 낫다)
# =============================================================================

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool


SRC_STOP_NO_AVOID = 'STOP(회피입력 없음)'
SRC_STOP_NO_TASK = 'STOP(임무게이트 없음)'
SRC_STOP_TASK_HOLD = 'STOP(임무 정지)'
SRC_AVOID = 'AVOID'
SRC_TASK = 'TASK'
SRC_SERVO = 'SERVO'
SRC_AVOID_CRUISE = 'AVOID(전진)'
SRC_STOP_IDLE = 'STOP(입력 없음)'


class CmdMuxNode(Node):

    def __init__(self):
        super().__init__('cmd_mux_node')

        self.declare_parameter('avoid_topic', '/cmd_vel_avoid')
        self.declare_parameter('servo_topic', '/cmd_vel_servo')
        #  서보가 통제권을 주장하는지 알려주는 토픽.
        self.declare_parameter('servo_active_topic', '/drain_servo_node/active')
        self.declare_parameter('out_topic', '/cmd_vel')

        #  ── TaskManager 접점 (2026-08-23 신설) ──
        #  게이트 둘은 '허가'이지 '명령'이 아니다. TaskManager 는 /cmd_vel 을
        #  발행하지 않는다 — 발행자는 여전히 이 노드 하나뿐이다.
        self.declare_parameter('allow_drive_topic', '/task/allow_drive')
        self.declare_parameter('servo_enable_topic', '/task/servo_enable')
        #  정밀 동작(50cm 전진)만 이 경로로 온다. 평소에는 아무것도 안 온다.
        self.declare_parameter('task_cmd_topic', '/cmd_vel_task')

        #  입력이 이 시간 넘게 없으면 '끊겼다'고 본다.
        #  회피 노드는 /scan(약 7Hz)에 물려 돌아가므로 0.5s 는 프레임 3~4개다.
        self.declare_parameter('avoid_timeout', 0.5)
        self.declare_parameter('servo_timeout', 0.5)
        #  TaskManager 는 20Hz 로 게이트를 낸다. 0.5s 는 프레임 10개다.
        self.declare_parameter('task_timeout', 0.5)

        #  ★ 회피 입력을 필수로 볼 것인가 ★
        #  true  : 회피 입력이 없으면 무조건 정지 (바닥 주행 시 반드시 true)
        #  false : 서보만으로도 움직인다 (바퀴 띄운 상태의 벤치 시험용)
        self.declare_parameter('require_avoid', True)

        #  ★ 임무 게이트를 필수로 볼 것인가 ★  (require_avoid 와 같은 성격)
        #  false(기본) : 게이트가 없으면 '허용'으로 본다.
        #                TaskManager 없이 돌리던 오늘까지의 구성이 그대로 산다.
        #  true        : 게이트가 없으면 정지. **바닥 주행 · 시연에서는 true**
        self.declare_parameter('require_task', False)

        #  angular.z 가 이 값 이하이면 '직진'으로 본다. 부동소수 비교용 여유.
        self.declare_parameter('straight_eps', 0.05)

        self.declare_parameter('publish_rate', 20.0)

        p = self.get_parameter
        self.avoid_timeout = p('avoid_timeout').value
        self.servo_timeout = p('servo_timeout').value
        self.require_avoid = p('require_avoid').value
        self.task_timeout = p('task_timeout').value
        self.require_task = p('require_task').value
        self.eps = p('straight_eps').value

        self.avoid = None
        self.avoid_t = 0.0
        self.servo = None
        self.servo_t = 0.0
        #  기본값 False — active 를 한 번도 못 받았으면 '주장 없음'으로 본다.
        #  (서보 노드가 아예 없는 구성에서도 회피만으로 정상 동작해야 한다)
        self.servo_active = False

        #  게이트 기본값은 **허용**이다. 한 번도 못 받았으면 TaskManager 가
        #  아예 없는 구성으로 보고 오늘까지의 동작을 유지한다.
        #  없는 것을 '정지'로 해석하면 서보 단독 시험이 전부 막힌다.
        #  대신 바닥에서는 require_task: true 로 이 관용을 끈다.
        self.allow_drive = True
        self.servo_enable = True
        self.gate_t = 0.0
        self.task_cmd = None
        self.task_cmd_t = 0.0

        self.last_src = None

        self.create_subscription(Twist, p('avoid_topic').value, self._on_avoid, 10)
        self.create_subscription(Twist, p('servo_topic').value, self._on_servo, 10)
        self.create_subscription(
            Bool, p('servo_active_topic').value, self._on_servo_active, 10)
        self.create_subscription(
            Bool, p('allow_drive_topic').value, self._on_allow_drive, 10)
        self.create_subscription(
            Bool, p('servo_enable_topic').value, self._on_servo_enable, 10)
        self.create_subscription(
            Twist, p('task_cmd_topic').value, self._on_task_cmd, 10)
        self.pub = self.create_publisher(Twist, p('out_topic').value, 10)
        self.src_pub = self.create_publisher(String, '~/source', 10)

        self.create_timer(1.0 / max(1.0, p('publish_rate').value), self._tick)

        if not self.require_task:
            self.get_logger().warn(
                'require_task=false — 임무 게이트가 없어도 주행한다. '
                '★바닥 주행과 시연에서는 true 로 줄 것★')
        if not self.require_avoid:
            self.get_logger().warn(
                'require_avoid=false — 회피 입력 없이도 주행한다. '
                '★바퀴를 띄운 상태에서만 쓸 것★')
        #  ★ 두 안전 스위치를 시작 로그에 함께 찍는다 ★
        #  require_task 는 false 일 때만 경고로 나오던 탓에 **true 로 켜고 띄우면
        #  로그에 아무 흔적이 없었다** — 시연에서 정말 켜졌는지 확인할 방법이 없었다.
        #  켜졌든 꺼졌든 값이 보여야 사후에 로그로 대조할 수 있다.
        self.get_logger().info(
            f"CmdMux 시작 · {p('avoid_topic').value} + {p('servo_topic').value} "
            f"-> {p('out_topic').value} · require_avoid={self.require_avoid} "
            f"· require_task={self.require_task}")

    # =====================================================================
    def _on_avoid(self, msg):
        self.avoid, self.avoid_t = msg, self._now()

    def _on_servo(self, msg):
        self.servo, self.servo_t = msg, self._now()

    def _on_servo_active(self, msg):
        self.servo_active = bool(msg.data)

    def _on_allow_drive(self, msg):
        self.allow_drive, self.gate_t = bool(msg.data), self._now()

    def _on_servo_enable(self, msg):
        self.servo_enable = bool(msg.data)

    def _on_task_cmd(self, msg):
        #  신선함만으로 '지금 TaskManager 가 직접 몰고 있다'를 판단한다.
        #  별도 활성 플래그를 두지 않는 이유: 플래그와 명령이 어긋날 여지를
        #  아예 만들지 않으려는 것이다. 안 몰 때는 발행 자체를 멈춘다.
        self.task_cmd, self.task_cmd_t = msg, self._now()

    def _is_cruising(self, t: Twist) -> bool:
        """회피 노드가 '길이 열렸다'고 판단한 상태인가."""
        return t.linear.x > 0.0 and abs(t.angular.z) <= self.eps

    # =====================================================================
    def _tick(self):
        now = self._now()
        avoid_ok = self.avoid is not None and (now - self.avoid_t) <= self.avoid_timeout
        servo_ok = self.servo is not None and (now - self.servo_t) <= self.servo_timeout
        gate_ok = (now - self.gate_t) <= self.task_timeout
        task_ok = (self.task_cmd is not None
                   and (now - self.task_cmd_t) <= self.task_timeout)

        # ⓪-a 임무 게이트가 필수인데 없다 -> 정지
        #     require_task=false 면 게이트 부재를 '허용'으로 본다(오늘까지의 구성).
        if self.require_task and not gate_ok:
            self._emit(Twist(), SRC_STOP_NO_TASK)
            return

        # ⓪-b TaskManager 가 명시적으로 '서지 마라'가 아니라 '서라'고 했다 -> 정지
        #     게이트가 신선할 때만 본다. 낡은 false 로 영영 서 있지 않게.
        if gate_ok and not self.allow_drive:
            self._emit(Twist(), SRC_STOP_TASK_HOLD)
            return

        # ① 회피 입력 두절 -> 정지
        if self.require_avoid and not avoid_ok:
            self._emit(Twist(), SRC_STOP_NO_AVOID)
            return

        # ② 회피가 회피 중이면 회피가 이긴다
        #    ★ TASK 보다 위다 ★ 임무 목표보다 안전이 우선이다.
        if avoid_ok and not self._is_cruising(self.avoid):
            self._emit(self.avoid, SRC_AVOID)
            return

        # ③ TaskManager 가 직접 명령 중이면 서보보다 우선한다
        #    (50cm 전진 같은 정밀 동작. 평소에는 아무것도 안 온다)
        if task_ok:
            self._emit(self.task_cmd, SRC_TASK)
            return

        # ④ 길이 열려 있다 -> 서보가 **통제권을 주장할 때만** 이긴다
        #    servo_enable 은 TaskManager 가 서보를 잠시 재우는 스위치다.
        #    꺼져도 아래 ⑤ 로 떨어져 회피의 전진은 그대로 산다.
        if servo_ok and self.servo_active and self.servo_enable:
            self._emit(self.servo, SRC_SERVO)
            return

        # ⑤ 서보가 없으면 회피의 전진을 그대로 통과시킨다 (탐색 주행)
        if avoid_ok:
            self._emit(self.avoid, SRC_AVOID_CRUISE)
            return

        self._emit(Twist(), SRC_STOP_IDLE)

    def _emit(self, cmd: Twist, src: str):
        self.pub.publish(cmd)
        self.src_pub.publish(String(data=src))
        if src != self.last_src:
            self.get_logger().info(f'출처 전환 -> {src}')
            self.last_src = src

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main():
    rclpy.init()
    node = CmdMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        #  ★ 종료 시 정지 명령을 내보낸다 ★
        #  이 노드가 죽으면 /cmd_vel 발행자가 사라지고, can_manager 는
        #  cmd_vel 타임아웃(0.3s)으로 IDLE 로 떨어진다. 그래도 명시적으로 0 을 보낸다.
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
