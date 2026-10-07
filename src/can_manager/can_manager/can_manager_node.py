#!/usr/bin/env python3
# =============================================================================
#  can_manager_node.py  —  ROS2 <-> CAN 게이트웨이 노드
# =============================================================================
#
#  [이 노드의 역할]
#    ROS2 토픽과 CAN 프레임 사이의 **유일한 통로**.
#    "무엇을 할지"는 TaskManager가 정하고, 이 노드는 "어떻게 전달할지"만 담당한다.
#
#  [인터페이스 요약]
#    구독   /cmd_vel           geometry_msgs/Twist       -> 0x200 RPM 필드
#           /emergency_stop    std_msgs/Bool             -> 0x200(EMERGENCY)
#
#    서비스 /clear_motor_fault robot_interfaces/ClearMotorFault -> 0x200 Fault Clear
#
#    발행   /motor/status      robot_interfaces/MotorStatusArray  <- 0x210 / 0x211
#           /battery/state     sensor_msgs/BatteryState           <- 0x310
#           /tof/status        robot_interfaces/TofStatus         <- 0x410
#           /can/health        robot_interfaces/CanHealth         (진단)
#           /odom + TF         nav_msgs/Odometry                  (실제 RPM 적분)
#
#  [설계 원칙 — 수정할 때 지켜야 할 것]
#    1. 판단 로직은 넣지 않는다. 변환 / 전송 / 안전 폴백만 담당한다.
#       예) TOF 거리로 회피할지 말지는 TaskManager 몫. 여기선 그대로 릴레이만.
#    2. 수신은 전용 스레드, 송신은 ROS2 타이머로 분리한다.
#       bus.recv()가 블로킹 함수라 콜백 안에서 부르면 executor 전체가 멈춘다.
#    3. 공유 상태는 반드시 self._lock 안에서만 만지되, 락 구간은 최소로 유지한다.
#       (파싱·발행은 락 밖에서, 변수 대입만 락 안에서)
#    4. 명령이 끊기면 무조건 정지 프레임을 보낸다. 상위가 죽어도 로봇은 멈춰야 한다.
#
#  [자주 수정하게 될 곳]
#    - 송신 주기 / 타임아웃  -> SECTION 2 (_declare_params) 또는 config/can_params.yaml
#    - 안전 정지 조건        -> SECTION 7 (_tx_motor)
#    - 기구 제원 (바퀴 등)   -> config/can_params.yaml
#
# =============================================================================

import math
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from robot_interfaces.msg import (
    MotorStatus, MotorStatusArray, CanHealth, TofStatus
)
from robot_interfaces.srv import ClearMotorFault
from can_manager import can_protocol as proto

try:
    import can  # python-can
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "python-can 이 필요합니다.  pip install python-can --break-system-packages"
    ) from exc


class CanManagerNode(Node):

    # =========================================================================
    #  SECTION 1. 초기화
    # =========================================================================
    #  [초기화 순서가 곧 안전 설계다]
    #    파라미터 -> 내부 상태 -> CAN 버스 -> ROS 인터페이스 -> 타이머 -> RX 스레드
    #
    #    * 구독보다 상태 변수를 먼저 만들어야 한다. 반대로 하면 create_subscription
    #      직후 /cmd_vel이 날아왔을 때 self._target_rpm이 없어서 AttributeError.
    #    * RX 스레드를 맨 마지막에 띄워야 한다. 먼저 띄우면 self.pub_motor가
    #      없는 상태에서 프레임이 들어올 수 있다.
    # -------------------------------------------------------------------------
    def __init__(self):
        super().__init__('can_manager')

        # --- 파라미터 읽기 ---------------------------------------------------
        self._declare_params()
        p = self.get_parameter
        self.iface            = p('can_interface').value
        self.wheel_radius     = p('wheel_radius').value
        self.wheel_separation = p('wheel_separation').value
        self.gear_ratio       = p('gear_ratio').value
        self.max_rpm          = p('max_rpm').value
        self.invert_left      = p('invert_left').value
        self.invert_right     = p('invert_right').value
        self.cmd_vel_timeout  = p('cmd_vel_timeout').value
        self.motor_timeout    = p('motor_status_timeout').value
        self.battery_timeout  = p('battery_status_timeout').value
        self.tof_timeout      = p('tof_status_timeout').value
        self.publish_odom     = p('publish_odom').value
        self.publish_tf       = p('publish_tf').value
        self.auto_recover     = p('auto_recover_bus_off').value

        # --- 내부 상태 -------------------------------------------------------
        #  아래 변수들은 RX 스레드와 ROS2 executor가 함께 접근한다.
        #  반드시 self._lock 안에서만 읽고 쓸 것.
        self._lock = threading.Lock()
        self._running = True

        # 송신할 명령 상태
        self._target_rpm = (0, 0)      # (좌, 우) 목표 RPM
        self._last_cmd_time = 0.0      # 마지막 /cmd_vel 수신 시각
        self._estop = False
        self._fault_clear_bits = 0     # 대기 중인 Fault Clear 비트
        self._fault_clear_pulses = 0   # 몇 프레임 더 반복 송신할지

        # 수신한 상태 + 마지막 수신 시각 (신선도 판정용)
        self._motor = {}               # motor_id -> (MotorStatusFrame, 수신시각)
        self._last_motor_rx = 0.0
        self._last_battery_rx = 0.0
        self._last_tof_rx = 0.0

        # 진단 카운터
        self._rx_count = 0
        self._tx_count = 0
        self._err_count = 0
        self._busoff_recover_count = 0
        self._bus_ok = False

        # 오도메트리 적분 상태
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_th = 0.0
        self._odom_last = time.time()

        # --- CAN 버스 오픈 ---------------------------------------------------
        self._bus = None
        self._open_bus()

        # --- ROS 인터페이스 --------------------------------------------------
        #  cmd_qos    : 명령은 유실되면 안 되므로 RELIABLE, 최신값만 필요해 depth=1
        #  sensor_qos : 센서 데이터는 조금 유실돼도 최신성이 중요하므로 BEST_EFFORT
        cmd_qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        sensor_qos = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, cmd_qos)
        self.create_subscription(Bool, 'emergency_stop',
                                 self._on_estop, cmd_qos)

        self.create_service(ClearMotorFault, 'clear_motor_fault',
                            self._on_clear_fault)

        self.pub_motor   = self.create_publisher(MotorStatusArray,
                                                 'motor/status', sensor_qos)
        self.pub_battery = self.create_publisher(BatteryState,
                                                 'battery/state', sensor_qos)
        self.pub_tof     = self.create_publisher(TofStatus,
                                                 'tof/status', sensor_qos)
        self.pub_health  = self.create_publisher(CanHealth, 'can/health', 10)
        self.pub_odom    = self.create_publisher(Odometry, 'odom', sensor_qos)
        self.tf_bc = TransformBroadcaster(self) if self.publish_tf else None

        # --- 송신 타이머 -----------------------------------------------------
        self.create_timer(p('tx_motor_period').value,    self._tx_motor)
        self.create_timer(0.05, self._update_odom)      # 오도메트리 적분 20Hz
        self.create_timer(1.0,  self._publish_health)   # 진단 1Hz

        # --- 수신 스레드 기동 (반드시 마지막) --------------------------------
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self.get_logger().info(
            f"CanManager 시작 · interface={self.iface} "
            f"· 휠반경={self.wheel_radius}m · 트레드={self.wheel_separation}m")

    # =========================================================================
    #  SECTION 2. 파라미터 선언
    # =========================================================================
    #  여기 기본값을 바꾸는 것보다 config/can_params.yaml을 고치는 쪽이 낫다.
    #  (YAML은 재빌드 없이 바로 반영됨)
    # -------------------------------------------------------------------------
    def _declare_params(self):
        # --- CAN 인터페이스 ---
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_bustype', 'socketcan')

        # --- 기구 제원 (실측값으로 반드시 교체할 것) ---
        self.declare_parameter('wheel_radius', 0.071)      # [m] 바퀴 반경 (지름 142mm, 2026-08-22)
        self.declare_parameter('wheel_separation', 0.40)   # [m] 좌우 바퀴 간격 (실측 약 40cm, 2026-08-22)
        self.declare_parameter('gear_ratio', 2.0)          # 모터:바퀴 감속비 (2026-08-22 육안 실측)
        self.declare_parameter('max_rpm', 300.0)           # 출력 상한

        # --- 배선에 따른 회전 방향 반전 ---
        #  모터 U/V/W 배선이나 홀센서 커넥터를 바꿔 꽂으면 방향이 뒤집힌다.
        #  코드 대신 이 값으로 잡을 것.
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)   # 2026-08-22 피드백 무시 결정으로 False

        # --- 안전 타임아웃 [s] ---
        #  상대 ECU 송신 주기의 3배로 잡는다. 프레임 한두 개 유실됐다고
        #  곧바로 비상정지가 걸리는 오탐을 막기 위한 여유치.
        self.declare_parameter('cmd_vel_timeout', 0.3)
        self.declare_parameter('motor_status_timeout', 0.3)    # BLDC 100ms x3
        self.declare_parameter('battery_status_timeout', 3.0)  # BAT 1s x3
        self.declare_parameter('tof_status_timeout', 3.0)      # TOF 1.000s x3 (2026-08-25 실측)

        # --- 송신 주기 [s] ---
        #  0x200은 BLDC팀 확인 결과 100ms.
        #  [!] BLDC측 RX 워치독(미수신 시 자동 정지) 유무와 타임아웃은 아직 미확인.
        #      확인되면 이 값이 그보다 충분히 빠른지 검증할 것.
        self.declare_parameter('tx_motor_period', 0.1)

        # --- 오도메트리 ---
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # --- 기타 ---
        self.declare_parameter('auto_recover_bus_off', False)  # 커널 restart-ms 권장
        self.declare_parameter('stop_on_motor_fault', True)
        self.declare_parameter('fault_clear_pulse_frames', 5)  # 5 x 100ms = 500ms

    # =========================================================================
    #  SECTION 3. CAN 버스 관리 (오픈 / 송신 / bus-off 복구)
    # =========================================================================

    def _open_bus(self):
        """CAN 버스를 연다. 실패해도 예외를 던지지 않고 플래그만 내린다.

        노드가 죽어버리면 ros2 launch 전체가 흔들리고, 나중에 CAN이
        올라와도 자동 복구가 안 되기 때문. RX 루프가 0.5초마다 재시도한다.
        """
        bustype = self.get_parameter('can_bustype').value
        try:
            self._bus = can.interface.Bus(channel=self.iface, bustype=bustype)
            self._bus_ok = True
            self.get_logger().info(f"CAN 버스 오픈 성공: {self.iface}")
        except Exception as e:
            self._bus_ok = False
            self.get_logger().error(
                f"CAN 버스 오픈 실패 ({self.iface}): {e}\n"
                f"  인터페이스를 먼저 올리세요:\n"
                f"  $ sudo ip link set {self.iface} up type can \\\n"
                f"      bitrate 500000 sample-point 0.875 restart-ms 100")

    def _send(self, can_id: int, data: bytes):
        """CAN 프레임 1개 송신.

        timeout=0.01이 핵심이다. 버스에 아무도 없거나 종단저항이 빠져 있으면
        ACK를 못 받아 TX 버퍼가 찬다. 타임아웃이 없으면 send()가 무한 블로킹돼
        송신 타이머가 executor를 물어버린다. 10ms = 100ms 주기의 10%.
        """
        if self._bus is None or not self._bus_ok:
            return
        try:
            self._bus.send(can.Message(arbitration_id=can_id,
                                       data=data,
                                       is_extended_id=False), timeout=0.01)
            self._tx_count += 1
        except can.CanError as e:
            self._err_count += 1
            # 로그 폭주 방지: 50회당 1번만 출력
            if self._err_count % 50 == 1:
                self.get_logger().warn(f"CAN 송신 실패: {e}")
            self._check_bus_off()

    def _check_bus_off(self):
        """bus-off 감지 및 (옵션) 자동 복구.

        커널 레벨 자동 복구(ip link ... restart-ms 100)를 켜두는 것이 가장
        안정적이며, 이 함수는 그 위의 이중 안전장치다.
        기본값 auto_recover_bus_off=False 이므로 평소엔 동작하지 않는다.
        """
        try:
            state = getattr(self._bus, 'state', None)
            if state is not None and state == can.BusState.ERROR:
                return
        except Exception:
            pass

        if not self.auto_recover:
            return

        self.get_logger().error("bus-off 추정 — 인터페이스 재기동 시도")
        try:
            subprocess.run(['sudo', 'ip', 'link', 'set', self.iface, 'down'],
                           check=False, timeout=2)
            time.sleep(0.1)
            subprocess.run(['sudo', 'ip', 'link', 'set', self.iface, 'up',
                            'type', 'can', 'bitrate', '500000',
                            'sample-point', '0.875'],
                           check=False, timeout=2)
            self._busoff_recover_count += 1
        except Exception as e:
            self.get_logger().error(f"인터페이스 재기동 실패: {e}")

    # =========================================================================
    #  SECTION 4. 수신 스레드
    # =========================================================================

    def _rx_loop(self):
        """CAN 프레임 수신 전용 스레드 (daemon).

        [설계 의도]
          - recv(timeout=0.2) : 데이터를 기다리려는 게 아니라, 0.2초마다
            깨어나 self._running을 확인하기 위한 장치다. 타임아웃 없이
            블로킹하면 Ctrl+C를 눌러도 스레드가 안 죽는다.
          - _dispatch를 try/except로 감싼 이유: 잘못된 프레임 하나 때문에
            수신 스레드 전체가 죽으면 그 뒤로 아무것도 못 받는다.
        """
        while self._running and rclpy.ok():
            # 버스가 죽어 있으면 0.5초마다 재연결 시도
            if self._bus is None or not self._bus_ok:
                time.sleep(0.5)
                self._open_bus()
                continue

            try:
                msg = self._bus.recv(timeout=0.2)
            except Exception as e:
                self._err_count += 1
                self.get_logger().warn(f"CAN 수신 오류: {e}")
                time.sleep(0.1)
                continue

            if msg is None:
                continue          # 타임아웃 — 정상 동작
            if msg.is_error_frame:
                self._err_count += 1
                continue          # 에러 프레임은 카운트만 하고 버림

            self._rx_count += 1
            try:
                self._dispatch(msg)
            except Exception as e:
                self.get_logger().warn(
                    f"프레임 파싱 실패 id=0x{msg.arbitration_id:03X}: {e}")

    def _dispatch(self, msg):
        """수신 프레임을 CAN ID로 분류해 해당 파서·발행자로 보낸다.

        [락 사용 원칙]
          파싱은 락 밖 -> 변수 갱신만 락 안 -> 발행도 락 밖.
          publish()가 락 안에 있으면 그동안 송신 타이머가 대기하게 된다.

        [발행을 타이머로 미루지 않는 이유]
          지연 때문이다. TaskManager가 모터 Fault를 늦게 아는 건 안전 문제.
        """
        cid = msg.arbitration_id
        now = time.time()

        # --- 0x210 / 0x211 : BLDC 모터 상태 ---
        if cid in proto.MOTOR_STATUS_IDS:
            frame = proto.unpack_motor_status(cid, bytes(msg.data))
            with self._lock:
                self._motor[frame.motor_id] = (frame, now)
                self._last_motor_rx = now
            self._publish_motor_status()

        # --- 0x310 : BAT 배터리 상태 ---
        elif cid == proto.CAN_ID_BAT_STATUS:
            frame = proto.unpack_battery_status(bytes(msg.data))
            with self._lock:
                self._last_battery_rx = now
            self._publish_battery(frame)

        # --- 0x410 : TOF 거리 센서 ---
        elif cid == proto.CAN_ID_TOF_STATUS:
            frame = proto.unpack_tof_status(bytes(msg.data))
            with self._lock:
                self._last_tof_rx = now
            self._publish_tof(frame)

        # 그 외 ID는 무시 (다른 노드끼리의 통신일 수 있음)

    # =========================================================================
    #  SECTION 5. 구독 콜백 (ROS 토픽 수신)
    # =========================================================================
    #  콜백은 상태 변수만 갱신하고, 실제 송신은 타이머가 담당한다.
    #  (예외: E-stop은 즉시 송신 — 아래 참조)
    # -------------------------------------------------------------------------

    def _on_cmd_vel(self, msg: Twist):
        """주행 명령 수신. 물리 단위(m/s, rad/s) -> RPM 변환만 하고 저장."""
        rpm_l, rpm_r = self._twist_to_rpm(msg.linear.x, msg.angular.z)
        with self._lock:
            self._target_rpm = (rpm_l, rpm_r)
            self._last_cmd_time = time.time()

    def _on_estop(self, msg: Bool):
        """비상정지 수신.

        다른 명령과 달리 타이머를 기다리지 않고 콜백에서 직접 프레임을 쏜다.
        타이머를 기다리면 최악 100ms가 더 걸리는데, 1m/s면 그 사이 10cm를
        더 간다. 비상정지에서 10cm는 유의미한 거리다.
        """
        with self._lock:
            changed = self._estop != msg.data
            self._estop = msg.data

        if changed:
            self.get_logger().warn(
                f"E-stop {'활성화' if msg.data else '해제'}")

        if msg.data:
            self._send(proto.CAN_ID_MOTOR_CMD,
                       proto.pack_motor_cmd(0, 0, proto.MOTOR_MODE_EMERGENCY))

    def _on_clear_fault(self, request, response):
        """모터 Fault Clear 서비스.

        Fault Clear는 지속 상태가 아니라 일회성 요청이므로, 지정된 프레임 수
        (기본 5회 = 500ms)만 반복 송신하고 자동으로 꺼진다. 반복은 프레임
        유실 대비용.

        성공 여부는 /motor/status의 fault_code가 0으로 돌아오는지로 확인할 것.
        (BLDC가 별도 응답 프레임을 주지 않음)

        사용 예:
          ros2 service call /clear_motor_fault \\
              robot_interfaces/srv/ClearMotorFault "{clear_all: true}"
        """
        bits = 0
        if request.clear_m0:
            bits |= proto.MOTOR_CLEAR_M0
        if request.clear_m1:
            bits |= proto.MOTOR_CLEAR_M1
        if request.clear_all:
            bits |= proto.MOTOR_CLEAR_ALL

        pulses = self.get_parameter('fault_clear_pulse_frames').value
        with self._lock:
            self._fault_clear_bits = bits
            self._fault_clear_pulses = pulses

        self.get_logger().info(f"Fault Clear 요청: bits=0x{bits:02X}")
        response.success = (bits != 0)
        return response

    # =========================================================================
    #  SECTION 6. 기구학 변환
    # =========================================================================

    def _twist_to_rpm(self, v: float, w: float):
        """차동구동 역기구학:  (선속도, 각속도) -> (좌 RPM, 우 RPM)

        v_l = v - w * (트레드/2)     좌륜 선속도 [m/s]
        v_r = v + w * (트레드/2)     우륜 선속도 [m/s]
        rpm = v / (2*pi*r) * 60 * 감속비

        제자리 회전(v=0)이면 v_l = -v_r 이 되어 좌우가 반대로 돈다.
        """
        half = self.wheel_separation / 2.0
        v_l = v - w * half
        v_r = v + w * half

        k = 60.0 / (2.0 * math.pi * self.wheel_radius) * self.gear_ratio
        rpm_l = v_l * k
        rpm_r = v_r * k

        # [중요] 포화 시 좌우를 각각 자르면(min/max) 회전 반경이 왜곡된다.
        #        비율을 유지한 채 전체를 스케일해야 궤적이 보존된다.
        #        예) 좌400/우200, 상한300 -> 개별클램프는 300/200(비율 깨짐)
        #                                  -> 비율유지는 300/150(궤적 유지)
        peak = max(abs(rpm_l), abs(rpm_r))
        if peak > self.max_rpm and peak > 0:
            scale = self.max_rpm / peak
            rpm_l *= scale
            rpm_r *= scale

        # 배선에 따른 방향 반전은 마지막에 적용
        if self.invert_left:
            rpm_l = -rpm_l
        if self.invert_right:
            rpm_r = -rpm_r

        return int(round(rpm_l)), int(round(rpm_r))

    def _rpm_to_twist(self, rpm_l: float, rpm_r: float):
        """정기구학: (좌 RPM, 우 RPM) -> (선속도, 각속도). 오도메트리용.

        역기구학의 정확한 역순이다. invert를 **먼저** 되돌려야 대칭이 맞는다
        (송신에선 마지막에 적용했으므로).
        """
        if self.invert_left:
            rpm_l = -rpm_l
        if self.invert_right:
            rpm_r = -rpm_r

        k = (2.0 * math.pi * self.wheel_radius) / (60.0 * self.gear_ratio)
        v_l = rpm_l * k
        v_r = rpm_r * k

        return (v_l + v_r) / 2.0, (v_r - v_l) / self.wheel_separation

    # =========================================================================
    #  SECTION 7. 송신 타이머  ★ 이 노드에서 가장 중요한 부분 ★
    # =========================================================================

    def _tx_motor(self):
        """0x200 모터 명령 송신 (기본 100ms 주기).

        [안전 게이트]
          아래 조건 중 하나라도 걸리면 RPM을 0으로 덮어쓴다.
          TaskManager가 "전진하라"고 해도 여기서 거부할 수 있다는 뜻이다.

            E-stop            -> EMERGENCY (명확한 위험)
            모터 Fault        -> EMERGENCY (하드웨어 이상)
            cmd_vel 두절      -> IDLE      (통신 문제, 복구되면 자동 RUN)
            모터 상태 두절    -> IDLE      (통신 문제)

          [!] EMERGENCY가 Fault Clear 없이 자동으로 RUN 복귀되는지는 미확인.
              래치 상태라면 여기에 복귀 로직을 추가해야 한다.

        [cmd_vel 타임아웃이 중요한 이유]
          TaskManager가 크래시하면 마지막 명령이 그대로 남아 로봇이 계속
          달린다. 300ms 안에 새 명령이 없으면 정지시키는 게 이걸 막는다.
        """
        now = time.time()

        # --- 공유 상태를 한 번에 읽어온다 (락 구간 최소화) ---
        with self._lock:
            estop = self._estop
            rpm_l, rpm_r = self._target_rpm
            cmd_age = now - self._last_cmd_time

            # 부팅 직후(_last_motor_rx == 0)는 "두절"이 아니라 "아직 안 켜짐".
            # 이걸 stale로 치면 명령↔상태 데드락에 빠진다.
            motor_stale = (now - self._last_motor_rx) > self.motor_timeout \
                if self._last_motor_rx > 0 else False

            faulty = [(mid, f.fault_code)
                      for mid, (f, _) in self._motor.items()
                      if f.fault_code != proto.MOTOR_FAULT_NONE]

            # Fault Clear 요청이 대기 중이면 지정 프레임 수만큼 반복 송신
            if self._fault_clear_pulses > 0:
                fault_clear = self._fault_clear_bits
                self._fault_clear_pulses -= 1
                if self._fault_clear_pulses == 0:
                    self._fault_clear_bits = 0
            else:
                fault_clear = 0

        # --- 안전 조건 판정 -> 제어 모드 결정 ---
        mode = proto.MOTOR_MODE_RUN
        reason = None

        if estop:
            mode = proto.MOTOR_MODE_EMERGENCY
            reason = 'E-stop'
        elif faulty and self.get_parameter('stop_on_motor_fault').value:
            mode = proto.MOTOR_MODE_EMERGENCY
            names = ', '.join(f"M{mid}:{proto.get_motor_fault_name(code)}"
                              for mid, code in faulty)
            reason = f'모터 Fault ({names})'
        elif cmd_age > self.cmd_vel_timeout:
            mode = proto.MOTOR_MODE_IDLE
            reason = 'cmd_vel 타임아웃'
        elif motor_stale:
            mode = proto.MOTOR_MODE_IDLE
            reason = '모터 상태 수신 두절'

        if reason is not None:
            rpm_l = rpm_r = 0
            self.get_logger().warn(
                f"출력 차단: {reason} -> {proto.get_motor_mode_name(mode)}",
                throttle_duration_sec=2.0)

        self._send(proto.CAN_ID_MOTOR_CMD,
                   proto.pack_motor_cmd(rpm_l, rpm_r, mode, fault_clear))

    # =========================================================================
    #  SECTION 8. ROS 토픽 발행
    # =========================================================================

    def _publish_motor_status(self):
        """/motor/status 발행. 수신 즉시 호출된다 (타이머 아님)."""
        now = time.time()
        out = MotorStatusArray()
        out.header.stamp = self.get_clock().now().to_msg()

        with self._lock:
            for mid, (f, stamp) in sorted(self._motor.items()):
                m = MotorStatus()
                m.motor_id    = mid
                m.rpm         = int(f.rpm)
                m.current     = float(f.current_a)
                m.temperature = float(f.temperature_c)
                m.fault_code  = int(f.fault_code)
                m.state_flags = int(f.state_flags)
                m.stale       = (now - stamp) > self.motor_timeout
                out.motors.append(m)

        self.pub_motor.publish(out)

    def _publish_battery(self, f):
        """/battery/state 발행 (sensor_msgs/BatteryState 표준 타입)."""
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage     = float(f.voltage_v)
        msg.current     = float(f.current_a)
        msg.temperature = float(f.temperature_c)
        msg.percentage  = float(f.soc_percent) / 100.0
        msg.present     = True

        # [!] 프로토콜에 충전 방향 정보가 없어 상시 방전으로 간주한다.
        #     충전 도크를 연동하면 별도 신호로 충전 상태를 판별해야 한다.
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        # BAT ECU의 Error Interrupt를 health 필드로 매핑.
        # TaskManager는 이 필드만 봐도 이상 유무를 알 수 있다.
        msg.power_supply_health = (
            BatteryState.POWER_SUPPLY_HEALTH_GOOD if not f.error
            else BatteryState.POWER_SUPPLY_HEALTH_UNSPECIFIED_FAILURE)

        self.pub_battery.publish(msg)

    def _publish_tof(self, f):
        """/tof/status 발행.

        센서 배치도가 없어 방향 해석 없이 배열 순서 그대로 전달한다.
        방향별 회피 판단은 TaskManager 몫.
        """
        now = time.time()
        msg = TofStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.total_distance    = int(f.total_cm)
        msg.sensor_distances  = [int(v) for v in f.sensor_cm]

        with self._lock:
            msg.stale = (now - self._last_tof_rx) > self.tof_timeout \
                if self._last_tof_rx > 0 else False

        self.pub_tof.publish(msg)

    def _update_odom(self):
        """휠 오도메트리 적분 후 /odom + TF 발행 (20Hz 타이머).

        [한계] 홀센서 RPM 기반이라 바퀴 슬립을 보정하지 못한다.
               젖은 노면이나 배수구 격자 위에서 오차가 누적된다.
               최종적으로는 SLAM 보정이나 IMU 융합에 의존해야 한다.
        """
        if not self.publish_odom:
            return

        now = time.time()
        dt = now - self._odom_last
        self._odom_last = now

        # dt > 0.5 가드: 시스템이 잠깐 멈췄다 재개될 때(SD카드 IO 병목,
        # 추론 폭주 등) 위치가 순간이동하는 걸 막는다. 그 구간은 버린다.
        if dt <= 0 or dt > 0.5:
            return

        with self._lock:
            if 0 not in self._motor or 1 not in self._motor:
                return          # 양쪽 모터 상태가 다 있어야 계산 가능
            rpm_l = self._motor[0][0].rpm
            rpm_r = self._motor[1][0].rpm

        # 오일러 적분 (각도 먼저 갱신 후 위치 계산)
        v, w = self._rpm_to_twist(rpm_l, rpm_r)
        self._odom_th += w * dt
        self._odom_x  += v * math.cos(self._odom_th) * dt
        self._odom_y  += v * math.sin(self._odom_th) * dt

        stamp = self.get_clock().now().to_msg()
        odom_frame = self.get_parameter('odom_frame').value
        base_frame = self.get_parameter('base_frame').value
        q = _yaw_to_quat(self._odom_th)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = odom_frame
        od.child_frame_id = base_frame
        od.pose.pose.position.x  = self._odom_x
        od.pose.pose.position.y  = self._odom_y
        od.pose.pose.orientation = q
        od.twist.twist.linear.x  = v
        od.twist.twist.angular.z = w

        # [중요] 공분산을 0으로 두면 SLAM이 "완벽하게 정확한 측정"으로 해석해
        #        휠 오도메트리를 과신한다. 슬립이 큰 실외 노면에서는 신뢰도를
        #        낮게 잡는 게 맞다.
        od.pose.covariance[0]  = 0.05   # x
        od.pose.covariance[7]  = 0.05   # y
        od.pose.covariance[35] = 0.1    # yaw

        self.pub_odom.publish(od)

        if self.tf_bc is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = odom_frame
            t.child_frame_id = base_frame
            t.transform.translation.x = self._odom_x
            t.transform.translation.y = self._odom_y
            t.transform.rotation = q
            self.tf_bc.sendTransform(t)

    def _publish_health(self):
        """/can/health 진단 발행 (1Hz). 각 ECU의 프레임 신선도를 알린다."""
        now = time.time()
        h = CanHealth()
        h.header.stamp = self.get_clock().now().to_msg()
        h.bus_ok             = self._bus_ok
        h.rx_count           = self._rx_count
        h.tx_count           = self._tx_count
        h.error_count        = self._err_count
        h.bus_off_recoveries = self._busoff_recover_count

        with self._lock:
            h.motor_stale = (self._last_motor_rx == 0 or
                             (now - self._last_motor_rx) > self.motor_timeout)
            h.battery_stale = (self._last_battery_rx == 0 or
                               (now - self._last_battery_rx) > self.battery_timeout)
            h.tof_stale = (self._last_tof_rx == 0 or
                           (now - self._last_tof_rx) > self.tof_timeout)

        self.pub_health.publish(h)

        if h.motor_stale:
            self.get_logger().warn("BLDC ECU 상태 프레임 수신 두절",
                                   throttle_duration_sec=5.0)

    # =========================================================================
    #  SECTION 9. 종료 처리
    # =========================================================================

    def shutdown(self):
        """종료 시 반드시 정지 프레임을 남기고 나간다.

        Ctrl+C로 껐을 때 로봇이 마지막 속도로 계속 굴러가면 안 된다.
        3회 반복은 프레임 유실 대비. main()의 finally에서 호출되므로
        예외로 죽든 정상 종료든 항상 실행된다.
        """
        self._running = False

        for _ in range(3):
            self._send(proto.CAN_ID_MOTOR_CMD,
                       proto.pack_motor_cmd(0, 0, proto.MOTOR_MODE_EMERGENCY))
            time.sleep(0.02)

        if self._bus is not None:
            try:
                self._bus.shutdown()
            except Exception:
                pass

        self.get_logger().info("CanManager 안전 종료 완료")


# =============================================================================
#  SECTION 10. 유틸리티 / 엔트리포인트
# =============================================================================

def _yaw_to_quat(yaw: float) -> Quaternion:
    """2D 요각 -> 쿼터니언 (z, w만 사용)."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def main(args=None):
    rclpy.init(args=args)
    node = CanManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
