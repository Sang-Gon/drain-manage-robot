"""USB 웹캠 → /image_raw 발행 노드.

원본: 비전팀 my_opencv_tutorials/img_pub.py (다른 라파에서 개발)
바뀐 것: 노드 이름 img_publisher → camera_publisher, 토픽을 파라미터로 뺌.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from drain_vision.camera_util import as_device, open_capture


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        self.declare_parameter('width', 640)
        self.width = self.get_parameter('width').value
        self.declare_parameter('height', 480)
        self.height = self.get_parameter('height').value
        #  ★ 번호가 아니라 by-id 경로다 (2026-08-24) ★
        #  카메라가 두 대(C270 앞 · C920 뒤)가 되면서 정수는 실제 위험이 됐다.
        #  USB 열거 순서가 바뀌면 **앞뒤가 뒤바뀐다.** camera_util 주석 참조.
        #  정수를 넣어도 그대로 동작한다(옛 설정 호환).
        self.declare_parameter(
            'camera_device',
            '/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_200901010001-video-index0')
        self.camera_device = as_device(self.get_parameter('camera_device').value)
        self.declare_parameter('frame_rate', 10)
        self.frame_rate = self.get_parameter('frame_rate').value
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('frame_id', 'camera')
        self.frame_id = self.get_parameter('frame_id').value

        #  ★ [2026-08-30] 프레임이 안 올 때의 복구 ★
        #  부팅 직후에 **열리기는 하는데 프레임이 한 장도 안 오는** 상태를 봤다
        #  (2026-08-30 03:12 자동 시작: 열기 성공 후 "프레임 획득 실패"만 계속).
        #  이때 노드는 살아 있으므로 launch 의 respawn 이 못 잡는다 —
        #  ★ 죽지 않고 실패하는 것이 가장 나쁘다 ★ 임무는 탐지 없이 그냥 굴러간다.
        #  그래서 연속 실패가 쌓이면 장치를 다시 열고, 그래도 안 되면 스스로 죽는다.
        self.declare_parameter('reopen_after_failures', 20)   # 20회 = 10Hz 기준 2초
        self.reopen_after_failures = self.get_parameter('reopen_after_failures').value
        self.declare_parameter('exit_after_reopens', 3)       # 재연결 3회까지 시도
        self.exit_after_reopens = self.get_parameter('exit_after_reopens').value
        self.fail_count = 0
        self.reopen_count = 0

        image_topic = self.get_parameter('image_topic').value
        self.publisher = self.create_publisher(Image, image_topic, 10)

        self.cap = open_capture(self.camera_device)
        if not self.cap.isOpened():
            self.get_logger().error(
                "카메라를 열 수 없습니다: " + str(self.camera_device))
            raise RuntimeError('VideoCapture open failed')

        # 웹캠이 직접 640x480 @ frame_rate 로 출력하도록 요청 (CPU resize 회피)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

        self.cv_bridge = CvBridge()
        time_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(time_period, self.time_callback)

        self.get_logger().info("Video Width : " + str(self.width))
        self.get_logger().info("Video Height : " + str(self.height))
        self.get_logger().info("Video Camera Device : " + str(self.camera_device))
        self.get_logger().info("Video Frame Rate : " + str(self.frame_rate))
        self.get_logger().info("Publish Topic : " + str(image_topic))
        self.get_logger().info(
            "Camera negotiated : %dx%d @ %.1ffps" % (actual_w, actual_h, actual_fps))
        if (actual_w, actual_h) != (self.width, self.height):
            self.get_logger().warn(
                "웹캠이 요청 해상도를 지원하지 않아 CPU resize로 대체합니다.")

    def _reopen(self):
        """장치를 닫았다 다시 연다. 한계를 넘기면 노드를 끝낸다(respawn 이 받는다)."""
        self.fail_count = 0
        self.reopen_count += 1
        if self.reopen_count > self.exit_after_reopens:
            #  ★ 여기서 죽는 것이 맞다 ★ 살아서 실패하면 아무도 눈치채지 못한다.
            self.get_logger().fatal(
                "카메라 재연결 %d회를 해도 프레임이 안 온다 — 노드를 끝낸다. "
                "launch 의 respawn 이 다시 띄운다: %s"
                % (self.exit_after_reopens, str(self.camera_device)))
            raise SystemExit(1)

        self.get_logger().warn(
            "프레임이 계속 안 온다 — 장치를 다시 연다 (%d/%d): %s"
            % (self.reopen_count, self.exit_after_reopens, str(self.camera_device)))
        try:
            self.cap.release()
        except Exception:                      # release 가 실패해도 재시도는 해야 한다
            pass
        self.cap = open_capture(self.camera_device)
        if not self.cap.isOpened():
            self.get_logger().error("재연결 실패 — 다음 주기에 다시 시도한다")
            return
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        self.get_logger().info("재연결 성공 — 프레임을 기다린다")

    def time_callback(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.fail_count += 1
            self.get_logger().warn(
                "프레임 획득 실패 — 이번 주기를 건너뜁니다. (연속 %d회)" % self.fail_count,
                throttle_duration_sec=2.0)
            if self.fail_count >= self.reopen_after_failures:
                self._reopen()
            return
        if self.fail_count:
            self.get_logger().info(
                "프레임 복귀 (연속 실패 %d회 뒤)" % self.fail_count)
            self.fail_count = 0
            self.reopen_count = 0
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        img = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = self.frame_id
        self.publisher.publish(img)


def main():
    rclpy.init()
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        if rclpy.ok():          # SIGTERM 등으로 이미 shutdown 된 경우 중복 호출 방지
            rclpy.shutdown()


if __name__ == '__main__':
    main()
