#!/usr/bin/env python3
# =============================================================================
#  tof_node — TOF 6개 값을 정리하고 '청소가 필요한가'를 판정한다
# =============================================================================
#  제공 서비스
#      ~/assess_drain   robot_interfaces/srv/AssessDrain
#      -> 실제 이름은 /tof_node/assess_drain
#         (TaskManager 의 assess_service 기본값과 같아야 한다)
#
#  ── ★ 이 노드는 CAN 을 건드리지 않는다 ★ ──
#    `can_manager` 가 이미 `0x410` 을 풀어 `/tof/status` 로 발행한다.
#    여기서 CAN 소켓을 또 여는 것은 **버스에 발행자를 늘리는 일**이고,
#    이 프로젝트가 `0x200` 으로 이미 한 번 사고를 낸 그 모양이다.
#    **읽기만 해도 마찬가지다 — 게이트웨이는 하나로 둔다.**
#
#  ── ★ 판정 규칙은 여기에 있다 ★ ──
#    TaskManager 는 '언제 물을지'만 안다. 철판 반사 제거 · 평균 · 문턱은
#    전부 이 노드의 파라미터다 (AssessDrain.srv 주석에 그렇게 적혀 있다).
#
#  ── 판정이 어떻게 나오는가 ──
#    ① 요청이 오면 /tof/status 를 sample_count 개 모은다 (sample_timeout 안에)
#    ② 센서별로 **표본들의 중앙값**을 뽑는다 — 한 프레임 튀는 것을 죽인다
#    ③ 그 6개에서 못 쓸 값을 버린다
#         · 0            = 측정 실패
#         · max_valid 초과 = 범위 밖 (혹은 포화)
#         · plate_reject 미만 = **배수구 구조물(그레이팅) 자체를 맞고 온 값**
#    ④ 남은 것의 평균을 낸다. min_valid_sensors 보다 적으면 UNKNOWN
#    ⑤ 평균이 needed_below_cm 보다 **짧으면** NEEDED (바닥이 얕아졌다 = 찼다)
#
#  ── ★ 철판 반사가 왜 '짧은 값'인가 ★ ──
#    센서는 배수구를 내려다본다. 그레이팅(철판)은 센서 바로 아래에 있고,
#    구멍 사이로는 그보다 **깊은 곳**이 보인다. 철판을 맞은 빔은 **짧게** 찍힌다.
#    쓰레기는 철판보다 아래에 쌓이므로 **철판보다는 멀고 빈 배수구보다는 가깝다.**
#    그래서 문턱 두 개가 이 순서여야 한다:
#        plate_reject_below_cm  <  needed_below_cm  <  max_valid_cm
#    어긋나면 기동 시 경고한다. **어긋난 채로 두면 영영 NEEDED 가 안 나온다.**
#
#  ── ★ [2026-08-25] 문턱값이 사용자 규격으로 확정됐다 ★ ──
#    TOF 보드가 붙었고 0x410 이 실제로 들어온다. 문턱은 사용자가 규격으로 줬다:
#      · 센서 하나가 **5cm 미만**이면 배수구 구조물 자체를 본 것 -> 그 센서만 버린다
#      · 남은 센서들의 **평균이 20cm 미만**이면 청소 필요(NEEDED)
#    평균의 아래끝 5cm 은 따로 안 넣었다 — 5cm 미만을 센서별로 먼저 버리므로
#    남은 값이 전부 5 이상이고 그 평균도 반드시 5 이상이다. 자동으로 지켜진다.
#
#    ★ 다만 실물 배수구에 대고 재 본 값은 아니다 ★ 확인할 것:
#      1. **깨끗한 배수구** 위에서 평균이 20 **위**로 나오는가
#      2. 청소가 필요할 만큼 **찬 배수구**에서 평균이 20 **아래**로 나오는가
#      3. 구조물(그레이팅) 반사가 정말 5 아래로 찍히는가
#    `simulate_distances` 로 값을 넣어 로직만 따로 확인할 수 있다(시험 전용).
#
#  ── ★ [2026-08-25 실측] 0x410 주기는 1.000초다 ★ ──
#    코드 곳곳의 '100ms 가정'은 틀렸다. 실 can0 32초에 32프레임(998~1003ms).
#    표본 수·타임아웃·stale 문턱이 전부 이 위에 얹혀 있었으므로 같이 고쳤다.
# =============================================================================

import statistics
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from robot_interfaces.msg import TofStatus
from robot_interfaces.srv import AssessDrain


V_UNKNOWN = 0
V_NOT_NEEDED = 1
V_NEEDED = 2
V_NAME = {0: 'UNKNOWN', 1: 'NOT_NEEDED', 2: 'NEEDED'}


class TofNode(Node):

    def __init__(self):
        super().__init__('tof_node')
        p = self.declare_parameter

        p('status_topic', '/tof/status')

        # ── 표본 ─────────────────────────────────────────────────────────
        #  ★ [2026-08-25 실측] 0x410 은 100ms 가 아니라 1.000초 주기다 ★
        #  실 can0 32초에 32프레임 · 간격 998.0~1003.2ms.
        #  1Hz 에서 표본 2개의 '중앙값'은 두 값의 평균이라 튄 프레임이
        #  절반의 무게로 그대로 들어온다. 3개여야 실제로 죽는다(최악 3.1초).
        #  TaskManager 는 이 동안 로봇을 세워 둔다(MEASURING 게이트가 정지).
        p('sample_count', 3)
        p('sample_timeout', 3.5)   # 이 안에 못 모으면 모인 것으로 판정
        #  마지막 수신이 이보다 오래됐으면 '들어오고 있지 않다'로 본다.
        #  1Hz 이므로 옛 0.5 는 정상 수신 중에도 두절로 읽히는 값이었다.
        p('stale_timeout', 2.5)

        # ── 문턱 (2026-08-25 사용자 규격 · 실물 대조는 아직) ──────────────
        p('plate_reject_below_cm', 12.0)   # 12cm 미만 = 배수구 구조물 -> 버린다
        #  ★ [2026-09-05] 20.0 -> 30.0 (사용자 결정) ★ yaml 과 같이 바꾼다 —
        #  파라미터 파일 없이 `ros2 run` 으로 띄우는 경로가 여기 값을 쓴다.
        #  yaml 만 고치면 그 경로가 조용히 옛 문턱으로 판정한다(08-29 모델 교체 때 겪은 일).
        p('needed_below_cm', 30.0)        # 유효 평균이 이보다 짧으면 청소 필요
        p('max_valid_cm', 200.0)          # 이보다 멀면 범위 밖 -> 버린다
        p('min_valid_sensors', 2)         # 유효 센서가 이보다 적으면 UNKNOWN

        # ── 시험 주입 ────────────────────────────────────────────────────
        #  ★ 시험 전용 ★ 비어 있지 않으면 /tof/status 를 보지 않고 이 값을 쓴다.
        #  TOF 보드 없이 판정 로직만 확인할 때. 쉼표로 6개.  예) "3,12,11,13,0,12"
        #  ★ 배열이 아니라 문자열인 이유 ★ rclpy 는 **빈 배열의 타입을 못 잡는다.**
        #  기본값 [] 로 선언하면 yaml 에서 [] 로 덮어쓰는 순간
        #  ParameterUninitializedException 이 난다(2026-08-24에 겪음).
        #  값을 주면 이번엔 BYTE_ARRAY 로 추론해 타입 충돌이 난다. 문자열이 안전하다.
        p('simulate_distances', '')

        g = self.get_parameter
        self.default_samples = int(g('sample_count').value)
        self.sample_timeout = float(g('sample_timeout').value)
        self.stale_timeout = float(g('stale_timeout').value)
        self.plate_cm = float(g('plate_reject_below_cm').value)
        self.needed_cm = float(g('needed_below_cm').value)
        self.max_cm = float(g('max_valid_cm').value)
        self.min_valid = int(g('min_valid_sensors').value)
        self.sim, sim_err = self._parse_sim(g('simulate_distances').value)

        self.samples = []          # [(t, [6개]), ...]
        self.last_rx = 0.0
        self.last_stale_flag = False
        self._lock = threading.Lock()

        #  ★ 콜백 그룹을 나눈다 ★
        #  판정은 '요청을 받은 뒤부터' 표본을 모아야 하므로 서비스 콜백이
        #  구독 콜백을 **기다린다.** 같은 그룹에 두면 서비스가 잡고 있는 동안
        #  구독이 못 돌아 **영원히 못 모으고 타임아웃**한다.
        #  (처음엔 콜백 안에서 spin_once 를 돌리려 했는데 그건 재진입이라 깨진다)
        #  두 그룹 + MultiThreadedExecutor 가 이 구조의 정답이다. main() 참조.
        self.cb_sub = MutuallyExclusiveCallbackGroup()
        self.cb_srv = MutuallyExclusiveCallbackGroup()

        self.create_subscription(TofStatus, g('status_topic').value,
                                 self._on_status, qos_profile_sensor_data,
                                 callback_group=self.cb_sub)
        self.create_service(AssessDrain, '~/assess_drain', self._on_assess,
                            callback_group=self.cb_srv)

        self.get_logger().info(
            f'TOF 노드 시작 · {g("status_topic").value} 구독 · '
            f'표본 {self.default_samples}개(최대 {self.sample_timeout:.1f}초) · '
            f'구조물 버림 <{self.plate_cm:.0f}cm · 필요 판정 <{self.needed_cm:.0f}cm · '
            f'범위 <={self.max_cm:.0f}cm · 최소 유효 {self.min_valid}개')

        # ── ★ 문턱 순서 검사 ★ ───────────────────────────────────────────
        #  어긋나면 판정이 조용히 한쪽으로 굳는다. 기동 때 말해 준다.
        if not (self.plate_cm < self.needed_cm < self.max_cm):
            self.get_logger().error(
                f'★ 문턱 순서가 어긋났다 ★ '
                f'plate({self.plate_cm}) < needed({self.needed_cm}) < '
                f'max({self.max_cm}) 여야 한다. '
                '이 상태로는 NEEDED 가 영영 안 나오거나 항상 나온다')
        self.get_logger().warn(
            '★ 문턱은 잠정값이다 (팀 시험 전) ★ 사용자 규격대로 넣었을 뿐 '
            '팀이 검증한 값이 아니다. 그리고 2026-08-25 실측에서 '
            '6개 중 1개만 반응했다 — 문턱보다 센서 생존을 먼저 볼 것 '
            '(~/tools/tof_live_check.py)')
        if self.sim is not None:
            self.get_logger().warn(
                f'simulate_distances={self.sim} — /tof/status 를 보지 않고 '
                '이 값으로 판정한다. ★시험 전용★')
        elif sim_err:
            self.get_logger().error(f'simulate_distances {sim_err} — 무시한다')

    @staticmethod
    def _parse_sim(raw):
        """"3,12,11,13,0,12" -> [3.0, ...]. 비었으면 (None, '')."""
        text = str(raw or '').strip()
        if not text:
            return None, ''
        parts = [x for x in text.replace(' ', '').split(',') if x]
        try:
            vals = [float(x) for x in parts]
        except ValueError:
            return None, f'숫자로 못 읽는다: "{text}"'
        if len(vals) != 6:
            return None, f'6개여야 한다(받은 것 {len(vals)}개)'
        return vals, ''

    # =====================================================================
    def _on_status(self, msg: TofStatus):
        now = self._now()
        with self._lock:
            self.last_rx = now
            self.last_stale_flag = bool(msg.stale)
            self.samples.append((now, [int(x) for x in msg.sensor_distances]))
            #  요청 없이도 계속 쌓이므로 넉넉히만 남긴다(최근 것만 쓴다)
            if len(self.samples) > 64:
                del self.samples[:-64]

    # =====================================================================
    def _on_assess(self, req, res):
        want = int(req.sample_count) or self.default_samples
        t0 = self._now()

        if self.sim is not None:
            per_sensor = list(self.sim)
            src = f'주입값(시험) {want}개 요청 무시'
            return self._judge(res, per_sensor, src, t0)

        #  ★ 지금부터 들어오는 것만 쓴다 ★
        #  주행 중에 쌓인 옛 값이 섞이면 '지금 이 배수구'의 판정이 아니게 된다.
        with self._lock:
            self.samples.clear()

        #  구독 콜백은 다른 그룹에서 계속 돈다(MultiThreadedExecutor).
        #  여기서는 자면서 기다리기만 한다.
        deadline = t0 + self.sample_timeout
        while self._now() < deadline:
            with self._lock:
                if len(self.samples) >= want:
                    break
            time.sleep(0.01)

        with self._lock:
            samples = list(self.samples)
            stale_flag = self.last_stale_flag
            last_rx = self.last_rx
        got = len(samples)
        if got == 0:
            age = (self._now() - last_rx) if last_rx else -1.0
            why = ('/tof/status 가 한 번도 온 적이 없다 (TOF ECU · CAN · '
                   'can_manager 확인)' if last_rx == 0.0 else
                   f'/tof/status 두절 (마지막 수신 {age:.1f}초 전)')
            return self._fail(res, why)

        if stale_flag:
            #  can_manager 가 스스로 '오래됐다'고 표시한 경우
            return self._fail(res, 'can_manager 가 TOF stale 로 표시했다')

        #  센서별 중앙값 — 한 프레임 튀는 값을 죽인다
        per_sensor = []
        for i in range(6):
            vals = [s[1][i] for s in samples]
            per_sensor.append(float(statistics.median(vals)))

        src = f'표본 {got}/{want}개'
        if got < want:
            src += f' (타임아웃 {self.sample_timeout:.1f}초)'
        return self._judge(res, per_sensor, src, t0)

    # =====================================================================
    def _judge(self, res, per_sensor, src, t0):
        valid, dropped = [], []
        for i, v in enumerate(per_sensor):
            if v <= 0:
                dropped.append(f'{i + 1}번 측정실패')
            elif v < self.plate_cm:
                #  파라미터 이름은 plate_reject_ 로 남아 있지만(그레이팅=철판에서 온 이름)
                #  사용자 규격의 표현은 '배수구 자체'다. 로그는 둘 다 덮는 말로 쓴다.
                dropped.append(f'{i + 1}번 구조물({v:.0f})')
            elif v > self.max_cm:
                dropped.append(f'{i + 1}번 범위밖({v:.0f})')
            else:
                valid.append(v)

        res.sensor_distances = [min(255, max(0, int(round(v))))
                                for v in per_sensor]
        res.valid_sensor_count = len(valid)

        if len(valid) < self.min_valid:
            res.verdict = V_UNKNOWN
            res.mean_distance_cm = 0.0
            res.detail = (f'유효 센서 {len(valid)}개 < 최소 {self.min_valid}개 — '
                          f'버림: {", ".join(dropped) or "없음"} · {src}')
            self.get_logger().warn(f'판정 UNKNOWN — {res.detail}')
            return res

        mean = sum(valid) / len(valid)
        res.mean_distance_cm = float(mean)
        res.verdict = V_NEEDED if mean < self.needed_cm else V_NOT_NEEDED
        res.detail = (
            f'평균 {mean:.1f}cm ({"<" if mean < self.needed_cm else ">="} '
            f'문턱 {self.needed_cm:.0f}) · 유효 {len(valid)}/6 · '
            f'버림: {", ".join(dropped) or "없음"} · {src} · '
            f'{self._now() - t0:.2f}초')
        self.get_logger().info(
            f'판정 {V_NAME[res.verdict]} — {res.detail}')
        return res

    def _fail(self, res, why):
        res.verdict = V_UNKNOWN
        res.mean_distance_cm = 0.0
        res.valid_sensor_count = 0
        res.sensor_distances = [0] * 6
        res.detail = why
        self.get_logger().warn(f'판정 UNKNOWN — {why}')
        return res

    def _now(self):
        return time.time()


def main():
    rclpy.init()
    node = TofNode()
    #  ★ 반드시 MultiThreadedExecutor 다 ★ 서비스 콜백이 구독 콜백을 기다리므로
    #  단일 스레드면 서로 막혀 표본이 영원히 안 모인다.
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
