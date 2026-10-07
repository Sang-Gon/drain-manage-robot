#!/usr/bin/env python3
"""TaskManager 의 ReportDrain 서비스를 server_bridge 의 토픽 규격으로 옮긴다.

    task_manager ──서비스 /comms_node/report_drain──> [이 노드]
                                                          │
                                        /drain_detection · /robot_status (String JSON)
                                                          ▼
                                                    server_bridge ──> MQTT · HTTP

★ 왜 어댑터인가 ★
  `server_bridge` 는 서버 팀이 준 코드다. **고치지 않는다**(사용자 지시).
  그쪽은 토픽 두 개를 구독하고, TaskManager 는 서비스를 부른다. 둘을 잇는 것이 이 노드다.
  TaskManager 도 안 고친다 — `report_service` 기본값이 이미
  `/comms_node/report_drain` 이라 **노드 이름 `comms_node` + 서비스 `~/report_drain`**
  이면 그대로 붙는다(`~` 가 없으면 `/report_drain` 이 되어 안 만난다).

★ 이 노드는 서버에 직접 나가지 않는다 ★
  MQTT 도 HTTP 도 열지 않는다. 토픽에 실어 `server_bridge` 에 넘길 뿐이다.
  **그래서 '서버가 받았는지'를 알 방법이 없다** — 아래 `success` 규약 참조.

사용
    ros2 run drain_comms comms_adapter_node --ros-args \
        --params-file ~/ros2_ws/install/drain_comms/share/drain_comms/config/comms_params.yaml
"""
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from robot_interfaces.srv import ReportDrain

#  ReportDrain.srv 와 같은 번호 (AssessDrain 과 공유한다)
V_UNKNOWN, V_NOT_NEEDED, V_NEEDED = 0, 1, 2
V_NAME = {V_UNKNOWN: 'UNKNOWN', V_NOT_NEEDED: 'NOT_NEEDED', V_NEEDED: 'NEEDED'}


class CommsAdapter(Node):
    def __init__(self):
        #  ★ 이름이 규약이다 ★ TaskManager 가 `/comms_node/report_drain` 을 부른다.
        #  이름을 바꾸면 task_params.yaml 의 `report_service` 도 같이 바꿔야 한다.
        super().__init__('comms_node')

        p = self.declare_parameter
        p('detection_topic', '/drain_detection')
        p('status_topic', '/robot_status')
        #  ★ 서버 규격 값 — 하드코딩하지 않는다 ★ (사용자 결정 2026-09-05)
        #  ★ drain_id 는 보고할 때마다 001 -> 010 으로 돌고 다시 001 로 온다 ★
        #  (사용자 결정 2026-09-05) 서버가 배수구를 열 칸으로 나눠 보게 된다.
        #  ★ 반드시 디스크에 남겨야 한다 ★ 이 노드는 임무마다 새로 뜨므로
        #  메모리에만 두면 **매 임무가 001 이 된다.** 그러면 도는 의미가 없다.
        #  ★ 붙임말이 규격이다 ★ server_bridge 의 규격 주석 예가 "D-003" 이다.
        #  2026-09-05 에 이걸 빼고 "001" 로 보냈다가 사용자가 잡았다.
        p('drain_id_prefix', 'D-')
        p('drain_id_first', 1)
        p('drain_id_last', 10)
        p('drain_id_digits', 3)
        p('state_file', '~/drain_comms_state.json')
        p('confidence', 1.0)
        #  ★ UNKNOWN 처리 ★ 서버 규격의 `full` 은 bool 이라 **판정 불가를 담을 자리가 없다.**
        #  'skip'       : 아예 안 보낸다 (기본 · 사용자 결정)
        #  'needed'     : needs_cleaning 으로 보낸다 (사람이 가서 본다)
        #  'not_needed' : normal 로 보낸다 — ★ 못 잰 것을 '깨끗했다'로 서버에 박는다. 쓰지 말 것 ★
        p('unknown_as', 'skip')
        p('status_period', 1.0)      # 0 이하면 상태를 안 보낸다
        p('battery_stale', 5.0)      # 이 시간 넘게 안 오면 battery 를 null 로 보낸다

        g = self.get_parameter
        self.id_prefix = str(g('drain_id_prefix').value)
        self.id_first = int(g('drain_id_first').value)
        self.id_last = int(g('drain_id_last').value)
        self.id_digits = int(g('drain_id_digits').value)
        self.state_path = os.path.expanduser(str(g('state_file').value))
        self.confidence = float(g('confidence').value)

        if self.id_last < self.id_first:
            self.get_logger().error(
                f'drain_id_last({self.id_last}) < first({self.id_first}) — '
                f'first 하나만 쓴다')
            self.id_last = self.id_first
        self.last_id = self._load_last_id()
        self.unknown_as = str(g('unknown_as').value).lower()
        self.batt_stale = float(g('battery_stale').value)

        if self.unknown_as not in ('skip', 'needed', 'not_needed'):
            self.get_logger().error(
                f"unknown_as 가 이상하다({self.unknown_as}) — 'skip' 으로 돌린다")
            self.unknown_as = 'skip'
        if self.unknown_as == 'not_needed':
            #  ★ 조용히 넘어가지 않는다 ★ 못 잰 것을 '깨끗했다'로 만드는 설정이다.
            self.get_logger().warn(
                "★ unknown_as=not_needed ★ 판정 불가를 서버에 '정상'으로 보낸다 — "
                "서버는 '깨끗함'과 '못 쟀음'을 구별할 수 없게 된다")

        #  server_bridge 는 depth 10 · 기본 RELIABLE 로 구독한다 — 같게 낸다.
        self.pub_det = self.create_publisher(String, g('detection_topic').value, 10)
        self.pub_st = self.create_publisher(String, g('status_topic').value, 10)

        #  ★ 센서 토픽은 BEST_EFFORT 다 ★ 기본 RELIABLE 로 걸면 0건이 온다(CLAUDE.md).
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(BatteryState, '/battery/state', self._on_batt, sensor_qos)
        self._batt_pct = None
        self._batt_t = 0.0

        #  ★ '~' 를 반드시 붙인다 ★ 상대 이름은 **노드 이름이 아니라 네임스페이스**
        #  기준으로 풀린다. 'report_drain' 이라고 쓰면 `/report_drain` 이 되어
        #  TaskManager 가 부르는 `/comms_node/report_drain` 과 **안 만난다.**
        #  2026-09-05 에 실제로 그렇게 만들었고 서비스 호출이 타임아웃으로 죽었다.
        self.srv = self.create_service(ReportDrain, '~/report_drain', self._on_report)

        period = float(g('status_period').value)
        if period > 0:
            self.create_timer(period, self._tick_status)

        self.get_logger().info(
            f'통신 어댑터 시작 · 서비스 /{self.get_name()}/report_drain → '
            f'{g("detection_topic").value} · drain_id '
            f'{self._fmt(self.id_first)}~{self._fmt(self.id_last)} 순환'
            f'(직전 {self._fmt(self.last_id) if self.last_id else "없음"}) · '
            f'confidence={self.confidence} · UNKNOWN={self.unknown_as} · '
            f'상태 {period}s')
        self.get_logger().info(
            '★ 이 노드는 서버에 직접 안 나간다 — server_bridge 가 MQTT/HTTP 를 맡는다 ★')

    # ── drain_id 순환 ────────────────────────────────────────────────────
    def _fmt(self, n):
        return self.id_prefix + str(n).zfill(self.id_digits)

    def _load_last_id(self):
        """★ 못 읽어도 죽지 않는다 ★ 통신이 임무를 멈추게 하지 않는다는 원칙 그대로.
        다만 조용히 넘어가지도 않는다 — 못 읽으면 처음부터 시작한다고 알린다."""
        try:
            with open(self.state_path, encoding='utf-8') as f:
                v = int(json.load(f)['last_drain_id'])
            if self.id_first <= v <= self.id_last:
                return v
            self.get_logger().warn(
                f'{self.state_path} 의 값 {v} 이 범위 밖이다 — 처음부터 시작한다')
        except FileNotFoundError:
            self.get_logger().info(f'{self.state_path} 없음 — 처음부터 시작한다')
        except Exception as e:
            self.get_logger().warn(
                f'{self.state_path} 를 못 읽었다({e}) — 처음부터 시작한다')
        return 0        # 0 이면 다음이 first 다

    def _next_drain_id(self):
        """★ 발행할 때만 부른다 ★ UNKNOWN 처럼 안 보내는 건은 번호를 안 쓴다 —
        쓰면 서버가 받는 번호에 구멍이 생긴다."""
        n = self.id_first if self.last_id <= 0 else self.last_id + 1
        if n > self.id_last:
            n = self.id_first
        self.last_id = n
        self._save_last_id(n)
        return self._fmt(n)

    def _save_last_id(self, n):
        #  ★ 임시 파일에 쓰고 rename 한다 ★ 이 로봇은 전원이 그냥 끊긴다.
        #  덮어쓰는 도중에 꺼지면 다음 기동에서 파일이 깨져 있다.
        tmp = self.state_path + '.tmp'
        try:
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump({'last_drain_id': n,
                           'time': time.strftime('%Y-%m-%dT%H:%M:%S')}, f)
            os.replace(tmp, self.state_path)
        except Exception as e:
            #  못 남겨도 이번 보고는 나간다. 다음 기동이 001 로 돌아갈 뿐이다.
            self.get_logger().error(f'{self.state_path} 저장 실패({e}) — '
                                    f'다음 기동에서 번호가 처음으로 돌아간다')

    # ── 배터리 ───────────────────────────────────────────────────────────
    def _on_batt(self, msg):
        self._batt_pct = round(msg.percentage * 100.0)
        self._batt_t = time.time()

    def _battery_or_none(self):
        """★ 옛 값을 살아 있는 값처럼 보내지 않는다 ★ 나이가 지나면 null 이다."""
        if self._batt_pct is None:
            return None
        if time.time() - self._batt_t > self.batt_stale:
            return None
        return self._batt_pct

    # ── /robot_status ────────────────────────────────────────────────────
    def _tick_status(self):
        #  ★ lat/lng 는 언제나 null 이다 ★ 이 로봇에 GPS 가 없다(사용자 결정).
        #  0.0 같은 그럴듯한 숫자를 넣으면 서버가 아프리카 앞바다를 가리킨다.
        payload = {
            'battery': self._battery_or_none(),
            'lat': None,
            'lng': None,
        }
        m = String()
        m.data = json.dumps(payload)
        self.pub_st.publish(m)

    # ── /drain_detection (ReportDrain 서비스) ────────────────────────────
    def _on_report(self, req, res):
        verdict = int(req.verdict)
        name = V_NAME.get(verdict, str(verdict))

        full = self._verdict_to_full(verdict)
        if full is None:
            #  ★ UNKNOWN 은 아예 안 보낸다 ★ (사용자 결정 2026-09-05)
            #  서버 규격의 `full` 은 bool 이라 '못 쟀다'를 담을 자리가 없다.
            #  거짓 데이터를 만드느니 데이터를 안 만든다. 로컬 근거는
            #  TaskManager 가 ~/drain_reports.jsonl 에 남긴다(sent:false).
            #  ★ 'full 필드만 빼는' 방식은 안 된다 ★ server_bridge 가
            #  `data.get('full', False)` 라 **필드를 빼면 조용히 normal 이 된다.**
            res.success = False
            res.server_message = f'판정 {name} — 서버 전송 생략 (규격에 판정 불가 자리 없음)'
            self.get_logger().warn(
                f'seq {req.sequence} · 판정 {name} → 전송 생략. '
                f'유효 {req.valid_sensor_count}/6 · 평균 {req.mean_distance_cm:.1f}cm')
            return res

        #  ★ 번호는 여기서 뽑는다 ★ 위의 '안 보냄' 분기를 지난 뒤라야
        #  보내지도 않고 번호만 축내는 일이 없다.
        drain_id = self._next_drain_id()
        payload = {
            'drain_id': drain_id,
            'full': full,
            'confidence': self.confidence,
            #  빈 문자열이면 server_bridge 가 업로드를 건너뛴다(`if photo_path`).
            'photo_path': req.image_path,
        }
        m = String()
        m.data = json.dumps(payload, ensure_ascii=False)
        self.pub_det.publish(m)

        #  ★ '보냈다'가 아니라 '넘겼다'까지만 답한다 ★
        #  server_bridge 는 결과를 돌려주지 않는다. 서버가 받았는지 알 수 없으므로
        #  아는 것만 말한다 — 받아 갈 노드가 붙어 있었는가.
        n = self.pub_det.get_subscription_count()
        if n < 1:
            res.success = False
            res.server_message = 'server_bridge 미기동 (구독자 0) — 전달 못 함'
            self.get_logger().error(
                f'seq {req.sequence} · {name} · drain_id={drain_id} 발행했으나 '
                '**구독자가 없다** — server_bridge 가 떠 있는지 볼 것')
        else:
            res.success = True
            res.server_message = 'server_bridge 로 전달 위임 (서버 응답은 확인 불가)'
            self.get_logger().info(
                f'seq {req.sequence} · {name} → drain_id={drain_id} · '
                f'full={full} · 사진 {req.image_path or "없음"} · 구독자 {n}')
        return res

    def _verdict_to_full(self, verdict):
        """서버의 bool `full` 로 옮긴다. 보내지 않을 때는 None 을 돌려준다."""
        if verdict == V_NEEDED:
            return True
        if verdict == V_NOT_NEEDED:
            return False
        #  UNKNOWN (또는 모르는 번호)
        if self.unknown_as == 'needed':
            return True
        if self.unknown_as == 'not_needed':
            return False
        return None


def main(args=None):
    rclpy.init(args=args)
    node = CommsAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
