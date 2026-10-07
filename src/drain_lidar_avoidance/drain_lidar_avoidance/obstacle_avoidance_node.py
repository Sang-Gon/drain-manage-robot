import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # [2026-08-19 실측] 기존 0.10 / 0.40 으로는 BLDC가 아예 기동하지 않았다.
        #   cmd_vel 스윕 결과 (바퀴 띄운 상태, can0 실차):
        #     0.10 m/s(≈11rpm) → state_flags=0x01, rpm=0, 전류 0.05A  … 정지
        #     0.20 m/s(≈22rpm) → state_flags=0x01, rpm=0, 전류 0.05A  … 정지
        #     0.30 m/s(≈34rpm) → state_flags=0x05                      … 회전 시작
        #   [2026-08-21 정정] 위 스윕은 0.10 m/s(약 11rpm) 단위라 거칠었다.
        #   1rpm 단위로 다시 재니 기동 하한은 34 가 아니라 **30rpm** 이다.
        #   29rpm 까지 flags=0x01(정지), 30rpm 부터 0x05(회전). 2회 반복 일치.
        #   유지 하한(돌던 것이 멈추는 값)도 30 으로 같았다 — 모터 쪽 히스테리시스 없음.
        #   그래서 0.35 m/s(≈39rpm)로 올려 12단계 시험을 통과했다.
        #
        # [2026-08-21] 사용자 요청으로 0.35 → 0.90 m/s(≈101rpm)로 상향.
        #   환산:  rpm = v / (2π·wheel_radius) × 60 = v × 112.35  (wheel_radius=0.085)
        #   can_params.yaml 의 max_rpm=300 아래라 클램프에 걸리지 않는다.
        #
        # ★★ [2026-08-22 오후] gear_ratio 가 1.0 -> 2.0 으로 확정됐다 ★★
        #   바퀴를 띄우고 회전수를 눈으로 센 결과 **바퀴는 명령의 절반으로 돈다**
        #   (명령 240 -> 실제 118 rpm, 비 0.492). 그래서 환산 상수가 또 바뀐다:
        #       rpm(명령) = v × 60/(2π·0.071) × 2.0 = v × 269.0
        #   현재 기본값이 실제로 만드는 명령값:
        #       forward_speed 0.90 -> 명령 242 rpm (실제 바퀴 121 rpm)
        #       reverse_speed 0.35 -> 명령  94 rpm (실제 바퀴  47 rpm)
        #
        #   ★ 실용 하한은 30rpm 이 아니라 **명령 60rpm** 이다 ★
        #   30rpm 명령은 기대(명령/2=15)의 47% 인 7rpm 밖에 안 나오고
        #   **사용자 육안으로 "뚝뚝 끊긴다". 60rpm 부터 부드럽다.**
        #   v 로는 0.223 m/s. forward/reverse 를 이 아래로 내리지 말 것.
        #
        #   ★★ [사용자 결정 2026-08-22] forward 0.90 -> 0.45, turn 4.50 -> 2.25 ★★
        #   이유: **21단계 실기 종합시험(4분·8배치)이 검증한 물리 속도가 0.45 m/s 였다.**
        #   그때 forward_speed=0.90 이라고 적혀 있었지만 2:1 을 몰랐던 탓이고,
        #   실제로 나간 것은 바퀴 60rpm(=0.45 m/s)이다.
        #   gear_ratio=2.0 을 넣은 지금 0.90 을 그대로 두면 **검증한 적 없는 2배 속도**가 된다.
        #   0.45 로 되돌리면 검증된 물리 상태를 그대로 재현한다.
        #   (속도를 다시 올리려면 max_rpm 상향을 함께 검토할 것)
        #
        #   ☆ 이 노드가 **실제로** 내보내는 세 가지 (can_manager 로 환산한 값) ☆
        #     전진  v=+0.45, w=0      -> 0x200  +121 / +121
        #     좌회전 v= 0.00, w=+2.25  -> 0x200  −121 / +121   (제자리 선회)
        #     우회전 v= 0.00, w=−2.25  -> 0x200  +121 / −121
        #     후진  v=−0.35, w=0      -> 0x200   −94 /  −94
        #   **넷 다 실용 하한 60rpm 위 · max_rpm=300 클램프에도 안 걸린다.**
        #
        #   reverse_speed 0.35 는 그대로 둔다 — 명령 94rpm 이라
        #   실용 하한 60rpm 위이고, 전진 0.45 보다 낮다는 설계 의도도 유지된다.
        #
        # ★ [2026-08-22 정정] wheel_radius 가 0.085 -> 0.071 로 확정됐다(지름 142mm 제원).
        #   환산 상수가 바뀐다:  rpm = v × 60 / (2π·0.071) = v × 134.50
        #   같은 파라미터 값이 이제 더 큰 rpm 을 만든다:
        #       forward_speed 0.90 -> 101rpm 이 아니라 ≈121rpm
        #       reverse_speed 0.35 ->  39rpm 이 아니라 ≈47rpm
        #   즉 파라미터를 그대로 둬도 로봇은 종전보다 약 20% 빨라진다.
        #   (종전이 틀렸던 것이다 — 0.90 m/s 라 적어놓고 실제로는 0.75 m/s 로 돌고 있었다)
        #   (이 절의 수치는 gear_ratio=1.0 전제라 위 오후 항목으로 대체됐다.)
        #
        #   turn_speed 는 **회전 시 바퀴 속력을 전진과 같게** 맞춘 값이다:
        #     w = 2·v / wheel_separation
        #   ★ [2026-08-22 재정정] 이 주석을 한때 "안쪽 바퀴가 0 이 되는 선회" 로
        #   고쳤었는데 **틀렸다. 코드를 안 읽고 고친 것이다.**
        #   아래 ② 분기가 `cmd.linear.x = 0.0` 이라 이 노드의 회전은 **제자리 선회**다.
        #   v=0 이면 v_l = −w·L/2, v_r = +w·L/2 로 좌우 대칭이고,
        #   w = 2·v_fwd/L 을 넣으면 바퀴 속력이 정확히 v_fwd 가 된다. 원래 주석이 옳았다.
        #
        # ★ [2026-08-22] wheel_separation 이 0.42 → 0.40 으로 실측됐다(약 40cm).
        #   그래서 w = 2·0.90 / 0.40 = 4.50 으로 맞춘다.
        #   **실기 동작은 사실상 바뀌지 않는다** — 옛 4.29 를 L=0.40 에 넣으면
        #   안쪽 바퀴가 0 이 아니라 0.042 m/s(≈6rpm)인데,
        #   이는 기동 하한 30rpm 아래라 ECU 가 어차피 0 으로 만든다.
        #   즉 옛 값도 결과적으로는 같은 선회였다. 의도와 숫자를 맞춰둔 것이다.
        #   (전진과 회전의 바퀴 rpm 을 같게 두어야 두 분기를 같은 조건에서 비교할 수 있다)
        #
        #   같은 세션에서 하드코딩을 걷어내고 ROS 파라미터로 뺐다.
        #   앞으로는 재빌드 없이 이렇게 튜닝한다:
        #     ros2 run drain_lidar_avoidance obstacle_avoidance_node \
        #         --ros-args -p forward_speed:=0.6 -p turn_speed:=3.00
        self.declare_parameter('safe_distance', 0.50)   # [m] 이 거리 안이면 정면이 막힌 것으로 본다
        self.declare_parameter('forward_speed', 0.45)   # [m/s] 전진 (명령 121rpm · 실제 바퀴 60rpm)
        self.declare_parameter('turn_speed', 2.25)      # [rad/s] 제자리 선회 (0x200 ∓121/±121)

        # [2026-08-21 추가] 후진 · 회전방향 유지
        #
        #   ★ 후진 속도를 전진보다 낮게 둔 이유 — 뒤가 안 보인다 ★
        #   섹터 정의상 front=±130~180° / left=−130~−50° / right=+50~+130° 이므로
        #   감시하지 않는 −49°~+49° 99도가 정확히 로봇 후방이다.
        #   즉 후진은 센서가 없는 방향으로 가는 동작이다.
        #   (실측 근거: 2026-08-21 16:09 기준값 측정에서 +6°에 0.34m 고정 반사가 있었다.
        #    감시 섹터 밖이라 판정에는 안 잡히지만 뒤에 뭔가 있었다는 뜻이다.)
        #   그래서 기본값을 기동 하한 위인 0.35 m/s 로 둔다.
        #   (기동 하한은 2026-08-21 1rpm 단위 실측으로 30rpm 확정.
        #    r=0.071 기준 0.35 m/s = 47rpm 이므로 하한의 1.57배 — 여유는 오히려 늘었다)
        #   올리려면 -p reverse_speed:=0.9 로 명시할 것.
        self.declare_parameter('reverse_speed', 0.35)   # [m/s] 후진 (명령 94rpm · 실제 바퀴 47rpm)

        #   좌우가 모두 이 거리 안이면 '옆도 막혔다'고 본다.
        self.declare_parameter('side_clear_distance', 0.30)   # [m]

        # [2026-08-21 추가] 후진 히스테리시스
        #
        #   처음 구현할 때 종료조건을 진입조건의 정확한 부정으로 뒀더니
        #   (front>0.50 or left>0.30 or right>0.30) 경계에서 후진이 떨었다.
        #   실측: left 가 0.31 -> 0.28 로 0.03m 흔들린 것만으로 후진이 4초 만에 재진입했고,
        #         front 0.67 -> 0.37 로도 6초 만에 재진입했다 (2026-08-21 17:00, 같은 세션 2회).
        #         물체는 그대로였다. 측정 잡음만으로 상태가 뒤집힌 것이다.
        #
        #   그래서 나가는 문턱을 들어오는 문턱보다 넓게 잡는다.
        #     진입: front <= 0.50  and  left <= 0.30  and  right <= 0.30
        #     종료: front >  0.60  or   left >  0.40  or   right >  0.40
        #   두 값 사이(0.50~0.60, 0.30~0.40)가 불감대다.
        #   후진 중이면 계속 후진하고, 후진 중이 아니면 평소 회피 로직이 돈다.
        self.declare_parameter('reverse_exit_front', 0.60)   # [m] 이보다 정면이 트이면 후진 종료
        self.declare_parameter('reverse_exit_side', 0.40)    # [m] 이보다 옆이 트이면 후진 종료

        #   좌우 거리 차가 이 값보다 작으면 직전 회전 방향을 유지한다.
        #   목적은 채터링 억제다 — left 와 right 가 엇비슷할 때 스캔마다(6.75Hz)
        #   좌/우가 뒤집히면 로봇이 제자리에서 떠는 것처럼 보인다.
        self.declare_parameter('direction_hold_margin', 0.10)  # [m]

        self.safe_distance = self.get_parameter('safe_distance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.reverse_speed = self.get_parameter('reverse_speed').value
        self.side_clear_distance = self.get_parameter('side_clear_distance').value
        self.reverse_exit_front = self.get_parameter('reverse_exit_front').value
        self.reverse_exit_side = self.get_parameter('reverse_exit_side').value
        self.direction_hold_margin = self.get_parameter('direction_hold_margin').value

        # 종료 문턱이 진입 문턱보다 좁으면 히스테리시스가 없다. 조용히 넘어가지 않는다.
        if (self.reverse_exit_front <= self.safe_distance
                or self.reverse_exit_side <= self.side_clear_distance):
            self.get_logger().warn(
                f'후진 종료 문턱이 진입 문턱보다 넓지 않다 — 경계에서 채터링한다. '
                f'진입 {self.safe_distance:.2f}/{self.side_clear_distance:.2f} · '
                f'종료 {self.reverse_exit_front:.2f}/{self.reverse_exit_side:.2f}'
            )

        # 상태 — 후진 중인가, 직전에 어느 쪽으로 돌았는가
        self.reversing = False
        self.last_turn = None   # 'left' | 'right' | None

        self.get_logger().info(
            f'LiDAR obstacle avoidance node started. '
            f'safe_distance={self.safe_distance:.2f}m '
            f'forward_speed={self.forward_speed:.2f}m/s '
            f'turn_speed={self.turn_speed:.2f}rad/s '
            f'reverse_speed={self.reverse_speed:.2f}m/s '
            f'side_clear={self.side_clear_distance:.2f}m '
            f'reverse_exit={self.reverse_exit_front:.2f}/{self.reverse_exit_side:.2f}m '
            f'hold_margin={self.direction_hold_margin:.2f}m'
        )

    def get_range_in_sector(self, scan_msg, angle_min_deg, angle_max_deg):
        valid_ranges = []

        for i, distance in enumerate(scan_msg.ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue

            angle_rad = scan_msg.angle_min + i * scan_msg.angle_increment
            angle_deg = math.degrees(angle_rad)

            while angle_deg > 180:
                angle_deg -= 360
            while angle_deg < -180:
                angle_deg += 360

            if angle_min_deg <= angle_deg <= angle_max_deg:
                if scan_msg.range_min <= distance <= scan_msg.range_max:
                    valid_ranges.append(distance)

        if len(valid_ranges) == 0:
            return float('inf')

        return min(valid_ranges)

    def scan_callback(self, scan_msg):
        # 측정 결과 반영:
        # 로봇 정면  = LiDAR 기준 -180도 / +180도 근처
        # 로봇 왼쪽  = LiDAR 기준 -90도 근처
        # 로봇 오른쪽 = LiDAR 기준 +90도 근처

        front_1 = self.get_range_in_sector(scan_msg, -180, -130)
        front_2 = self.get_range_in_sector(scan_msg, 130, 180)
        front = min(front_1, front_2)

        left = self.get_range_in_sector(scan_msg, -130, -50)
        right = self.get_range_in_sector(scan_msg, 50, 130)

        cmd = Twist()

        # ── ① 사방이 막혔는가 → 후진 (히스테리시스 있음) ────────────
        #  진입: front <= safe_distance      and left <= side_clear      and right <= side_clear
        #  종료: front >  reverse_exit_front or  left >  reverse_exit_side or  right >  reverse_exit_side
        #
        #  두 문턱 사이는 불감대다. 후진 중이면 계속 후진하고,
        #  후진 중이 아니면 아래 평소 로직이 돈다. 상태가 그 사이에서 뒤집히지 않는다.
        if self.reversing:
            escaped = (
                front > self.reverse_exit_front
                or left > self.reverse_exit_side
                or right > self.reverse_exit_side
            )
            if escaped:
                self.reversing = False
                self.get_logger().info(
                    f'Reverse end. front={front:.2f}m, left={left:.2f}m, right={right:.2f}m '
                    f'-> 회피 로직 복귀'
                )
        else:
            boxed_in = (
                front <= self.safe_distance
                and left <= self.side_clear_distance
                and right <= self.side_clear_distance
            )
            if boxed_in:
                self.reversing = True
                self.last_turn = None   # 후진 뒤에는 방향을 새로 판단한다
                self.get_logger().warn(
                    f'Boxed in -> reverse. front={front:.2f}m, '
                    f'left={left:.2f}m, right={right:.2f}m'
                )

        if self.reversing:
            # 양 바퀴 모두 뒤로. 후진 중에는 방향을 틀지 않는다.
            cmd.linear.x = -self.reverse_speed
            cmd.angular.z = 0.0

            self.get_logger().info(
                f'Reversing. front={front:.2f}m, left={left:.2f}m, right={right:.2f}m '
                f'-> back {self.reverse_speed:.2f}m/s'
            )
            self.cmd_pub.publish(cmd)
            return

        # ── ② 정면만 막혔는가 → 회전 ───────────────────────────────
        if front < self.safe_distance:
            cmd.linear.x = 0.0

            # 좌우가 엇비슷하면 직전 방향을 유지한다.
            #  left/right 가 둘 다 inf(그 섹터에 유효점 없음)이면 차이를 0 으로 본다.
            #  한쪽만 inf 면 차이가 inf 라 유지되지 않고 트인 쪽으로 간다.
            if math.isinf(left) and math.isinf(right):
                diff = 0.0
            else:
                diff = abs(left - right)

            if self.last_turn is not None and diff < self.direction_hold_margin:
                direction = self.last_turn
                reason = f'hold (|L-R|={diff:.2f}m < {self.direction_hold_margin:.2f}m)'
            else:
                direction = 'left' if left > right else 'right'
                reason = 'wider side'

            cmd.angular.z = self.turn_speed if direction == 'left' else -self.turn_speed
            self.last_turn = direction

            self.get_logger().info(
                f'Obstacle detected! front={front:.2f}m, left={left:.2f}m, right={right:.2f}m '
                f'-> turn {direction} [{reason}]'
            )

        # ── ③ 열려 있다 → 전진 ─────────────────────────────────────
        else:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
            #  전진으로 돌아오면 회전 이력을 지운다. 방향 유지는 '한 번의 회전 구간
            #  안에서만' 적용한다. 그래야 한참 뒤에 만난 다른 장애물에 옛 방향이
            #  끌려오지 않는다.
            self.last_turn = None

            self.get_logger().info(
                f'Path clear. front={front:.2f}m, left={left:.2f}m, right={right:.2f}m -> move forward'
            )

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
