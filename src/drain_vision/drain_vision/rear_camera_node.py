#!/usr/bin/env python3
# =============================================================================
#  rear_camera_node — 후방 카메라로 배수구를 캡쳐한다 (Logitech C920)
# =============================================================================
#  제공 서비스
#      ~/capture_drain   robot_interfaces/srv/CaptureDrain
#      -> 실제 이름은 /rear_camera_node/capture_drain
#         (TaskManager 의 capture_service 기본값과 같아야 한다)
#
#  ★ 영상을 서비스로 실어 나르지 않는다 ★
#    디스크에 저장하고 **경로만** 돌려준다. 통신 노드가 그 경로를 읽어 보낸다.
#    라파에서 영상을 두 번 나르는 것은 낭비다 (CaptureDrain.srv 주석 참조).
#
#  ── ★ 왜 기본이 '필요할 때 연다'인가 (keep_open=false) ★ ──
#    앞 카메라(C270)가 10Hz 로 계속 돌고 Hailo 가 그 위에서 추론한다.
#    후방 카메라는 배수구 하나당 **사진 한 장**을 위해 쓰인다.
#    그 한 장 때문에 USB 대역과 CPU 를 임무 내내 잡고 있을 이유가 없다.
#    **탐지 정확도가 이 프로젝트의 약한 고리**라(CLAUDE.md) 앞 카메라 쪽을
#    방해하지 않는 편을 골랐다.
#      실측: 열기 0.42초 + 워밍업 8프레임 -> **총 1.2초쯤**. 캡쳐 시점에는
#      로봇이 이미 서 있으므로(TaskManager 가 CAPTURING 에서 게이트를 닫는다)
#      이 정도 지연은 문제가 되지 않는다.
#    ★ keep_open=true 로 바꾸면 ★ 장치를 계속 열어 두고 최신 프레임을 들고 있다.
#      캡쳐는 즉시 끝나지만 USB 대역을 내내 쓴다. 캡쳐가 잦아지면 그때 켤 것.
#
#  ── ★ 워밍업 프레임을 버리는 이유 ★ ──
#    막 연 카메라의 첫 프레임들은 **자동노출·화이트밸런스가 수렴하기 전**이라
#    어둡거나 색이 튄다. 배수구 사진은 사람이 보고 판단할 물증이므로 버린다.
#
#  ── ★ 장치를 번호가 아니라 경로로 잡는다 ★ ──
#    `camera_device: 0` 같은 정수는 **USB 열거 순서가 바뀌면 앞뒤가 뒤바뀐다.**
#    앞 카메라를 후방으로 찍는 사고가 난다. 그래서 기본값을 by-id 경로로 뒀다:
#      /dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0
#    이 경로는 **모델명으로 만들어져 재부팅·재연결에도 유지된다.**
#    (앞 카메라 camera_publisher 는 아직 정수를 쓴다 — 미해결로 남아 있다)
#
#  ── 실패해도 임무를 멈추지 않는다 ──
#    카메라가 없거나 못 읽으면 **success=false 와 사유**를 돌려준다.
#    TaskManager 는 그 사유를 보고에 실어 서버로 보낸다.
#    노드가 아예 없는 것보다 **"왜 못 찍었는지"가 남는 편이 낫다.**
# =============================================================================

import os
import time

import cv2
import rclpy
from rclpy.node import Node

from robot_interfaces.srv import CaptureDrain

from drain_vision.camera_util import as_device, open_capture


class RearCameraNode(Node):

    def __init__(self):
        super().__init__('rear_camera_node')
        p = self.declare_parameter

        # ── 장치 ─────────────────────────────────────────────────────────
        #  ★ 번호가 아니라 경로다 ★ 위 주석 참조. 정수 문자열("2")도 받는다.
        p('camera_device',
          '/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0')
        #  C920 은 MJPG 로 1280x720@30 을 낸다(실측). YUYV 로 열리면 대역을
        #  훨씬 많이 먹고 프레임률이 떨어진다.
        p('fourcc', 'MJPG')
        p('width', 1280)
        p('height', 720)

        # ── 캡쳐 ─────────────────────────────────────────────────────────
        p('warmup_frames', 8)     # 자동노출 수렴 전 프레임을 버린다
        p('open_retry', 2)        # 열기 실패 시 재시도 횟수
        p('retry_delay', 0.3)
        #  true = 장치를 계속 열어 둔다(즉시 캡쳐 · USB 대역 상시 점유)
        p('keep_open', False)
        #  ★ 캡쳐 후 장치를 바로 닫지 않고 이만큼 뒤에 닫는다 ★
        #  cv2 의 release() 가 이 카메라에서 **약 2.0초** 걸린다(실측).
        #  서비스 콜백 안에서 닫으면 그 2초가 **응답 시간에 그대로 붙는다** —
        #  2026-08-24 에 캡쳐는 0.82초에 끝났는데 TaskManager 는 3.5초를 기다렸다.
        #  응답을 먼저 보내고 닫는다. 그 사이에 다음 캡쳐가 오면 열린 장치를
        #  그대로 재사용하므로 여는 시간(0.4초)도 아낀다.
        p('close_delay', 0.5)

        # ── 저장 ─────────────────────────────────────────────────────────
        p('save_dir', '~/drain_captures')
        p('jpeg_quality', 90)

        g = self.get_parameter
        self.dev = as_device(g('camera_device').value)
        self.fourcc = str(g('fourcc').value)
        self.width = int(g('width').value)
        self.height = int(g('height').value)
        self.warmup = int(g('warmup_frames').value)
        self.open_retry = int(g('open_retry').value)
        self.retry_delay = float(g('retry_delay').value)
        self.keep_open = bool(g('keep_open').value)
        self.close_delay = float(g('close_delay').value)
        self.save_dir = os.path.expanduser(str(g('save_dir').value))
        self.quality = int(g('jpeg_quality').value)

        self.cap = None
        self.count = 0
        #  0 이면 예약 없음. 그 외에는 '이 시각이 지나면 닫는다'.
        self._close_at = 0.0

        os.makedirs(self.save_dir, exist_ok=True)

        self.srv = self.create_service(CaptureDrain, '~/capture_drain',
                                       self._on_capture)
        #  ★ 늦은 닫기 담당 ★ 서비스 응답을 보낸 뒤에 장치를 닫는다.
        self.create_timer(0.2, self._tick_close)

        self.get_logger().info(
            f'후방 카메라 노드 시작 · 장치={self.dev} · '
            f'{self.width}x{self.height} {self.fourcc} · '
            f'워밍업 {self.warmup}프레임 · keep_open={self.keep_open} · '
            f'저장 {self.save_dir}')

        # ── 기동 시 한 번 확인한다 ───────────────────────────────────────
        #  ★ 캡쳐 순간에 처음 알게 되는 것이 제일 나쁘다 ★
        #  그때는 로봇이 이미 배수구를 지나쳐 서 있고 다시 못 찍는다.
        if self.keep_open:
            if self._open():
                self.get_logger().info('장치를 열어 둔 채 대기한다(keep_open)')
        else:
            ok, why = self._probe()
            if ok:
                self.get_logger().info('기동 점검 통과 — 카메라가 프레임을 준다')
            else:
                #  죽지 않는다. 서비스는 살아서 '왜 못 찍는지'를 돌려준다.
                self.get_logger().error(
                    f'★ 기동 점검 실패: {why} ★ 서비스는 계속 뜨지만 '
                    '캡쳐는 실패로 응답한다. 연결·권한을 확인할 것')

    # =====================================================================
    def _open(self):
        cap = open_capture(self.dev)
        if not cap.isOpened():
            cap.release()
            return False
        #  ★ 순서가 중요하다 ★ FOURCC 를 해상도보다 먼저 준다.
        #  나중에 주면 드라이버가 이미 잡은 포맷 위에서 협상해 MJPG 가 안 걸린다.
        if len(self.fourcc) == 4:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap = cap
        return True

    def _close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def _tick_close(self):
        if self._close_at and time.time() >= self._close_at:
            self._close_at = 0.0
            t0 = time.time()
            self._close()
            self.get_logger().debug(f'장치 닫음({time.time() - t0:.2f}초)')

    def _probe(self):
        """열어서 한 장 읽어 보고 닫는다. 기동 점검용."""
        try:
            if not self._open():
                return False, f'장치를 열 수 없다: {self.dev}'
            ok, frame = self.cap.read()
            if not ok or frame is None:
                return False, '장치는 열렸으나 프레임을 못 읽는다'
            return True, ''
        except Exception as e:
            return False, f'점검 중 예외: {e}'
        finally:
            if not self.keep_open:
                self._close()

    def _grab(self):
        """워밍업 프레임을 버리고 한 장을 돌려준다. (frame, 사유)"""
        frame = None
        for i in range(max(1, self.warmup)):
            ok, f = self.cap.read()
            if ok and f is not None:
                frame = f
            elif frame is None and i >= 2:
                #  앞쪽 몇 장은 실패해도 봐준다. 계속 실패하면 포기.
                return None, f'프레임 획득 실패({i + 1}번째)'
        if frame is None:
            return None, '프레임을 한 장도 못 읽었다'
        return frame, ''

    def _schedule_close(self, delay=None):
        """응답을 보낸 뒤에 닫도록 예약한다. keep_open 이면 안 닫는다."""
        if self.keep_open:
            return
        d = self.close_delay if delay is None else delay
        self._close_at = time.time() + d

    # =====================================================================
    def _on_capture(self, req, res):
        t0 = time.time()
        tag = ''.join(c for c in (req.tag or '')
                      if c.isalnum() or c in ('-', '_'))
        res.success = False
        res.image_path = ''
        res.width = 0
        res.height = 0

        #  닫기 예약이 걸려 있으면 취소한다. 아직 열려 있으면 그대로 쓴다.
        self._close_at = 0.0
        try:
            if self.cap is None:
                for attempt in range(self.open_retry + 1):
                    if self._open():
                        break
                    if attempt < self.open_retry:
                        self.get_logger().warn(
                            f'장치 열기 실패 — 재시도 {attempt + 1}/{self.open_retry}')
                        time.sleep(self.retry_delay)
                else:
                    #  ★ '없다'와 '남이 쓰고 있다'를 구별해 준다 ★
                    #  V4L2 는 이미 열려 있는 장치를 두 번 열어 주지 않는다.
                    #  2026-08-24 에 옛 시험의 노드가 살아남아 같은 서비스를
                    #  둘이 물었고, 장치를 못 연 쪽이 먼저 실패로 답해
                    #  **캡쳐가 성공했는데 실패로 보고됐다.** 그때 이 메시지가
                    #  "열 수 없다" 뿐이라 원인을 찾는 데 시간이 걸렸다.
                    exists = os.path.exists(self.dev) if isinstance(
                        self.dev, str) else True
                    res.detail = (
                        f'장치를 열 수 없다: {self.dev} — '
                        + ('장치 파일은 있다. **다른 프로세스가 이미 열고 있을 수 '
                           '있다**(rear_camera_node 가 두 개 떠 있는지 확인할 것)'
                           if exists else '장치 파일 자체가 없다(연결·전원 확인)'))
                    self.get_logger().error(f'캡쳐 실패 — {res.detail}')
                    return res

            frame, why = self._grab()
            if frame is None:
                res.detail = why
                self.get_logger().error(f'캡쳐 실패 — {why}')
                #  프레임을 못 읽는 상태로 장치를 붙들고 있어 봐야 소용없다.
                #  다음 호출에서 새로 열어 보게 놓아 준다.
                #  ★ 여기서 바로 닫지 않는다 ★ release() 가 2초라 응답이 그만큼 늦는다.
                self._schedule_close(0.0)
                return res

            h, w = frame.shape[:2]
            name = time.strftime('%Y%m%d_%H%M%S')
            if tag:
                name += f'_{tag}'
            path = os.path.join(self.save_dir, name + '.jpg')

            ok = cv2.imwrite(path, frame,
                             [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
            if not ok or not os.path.exists(path):
                res.detail = f'파일 저장 실패: {path}'
                self.get_logger().error(res.detail)
                self._schedule_close()
                return res

            self.count += 1
            size = os.path.getsize(path)
            res.success = True
            res.image_path = path
            res.width = w
            res.height = h
            res.detail = (f'{w}x{h} · {size / 1024:.0f}KB · '
                          f'{time.time() - t0:.2f}초')
            self.get_logger().info(
                f'캡쳐 #{self.count} 저장: {path} ({res.detail})')
            self._schedule_close()
            return res

        except Exception as e:
            res.detail = f'캡쳐 중 예외: {e}'
            self.get_logger().error(res.detail)
            self._schedule_close()
            return res


def main():
    rclpy.init()
    node = RearCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
