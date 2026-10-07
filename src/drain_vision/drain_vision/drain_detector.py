"""/image_raw → Hailo YOLOv11s 추론 → /detections/image 발행 노드.

원본: 비전팀 my_opencv_tutorials/yolo_detect.py (다른 라파에서 개발)
바뀐 것:
  - 패키지 이름 my_opencv_tutorials → drain_vision (share/models 경로 해석에 쓰임)
  - 노드 이름 yolo_detector → drain_detector, 토픽을 파라미터로 뺌
  - hailo_platform import 실패 시 설치 안내를 담은 메시지로 감쌈
    (Main ECU 라파는 2026-08-20 기준 Hailo 모듈 미장착. 장착 후엔 그대로 동작한다)
"""

import sys
import os
import site

# HailoRT 파이썬 휠은 보통 pip --user 로 깔려 ~/.local 에 들어간다.
# ros2 run 으로 띄우면 이 경로가 sys.path 에 없을 수 있어 직접 넣어준다.
user_site = site.getusersitepackages()
if user_site not in sys.path:
    sys.path.insert(0, user_site)

from contextlib import ExitStack

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from robot_interfaces.msg import DrainDetection, DrainDetectionArray
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np

try:
    from hailo_platform import (
        VDevice, HEF, ConfigureParams, HailoStreamInterface,
        InputVStreamParams, OutputVStreamParams, FormatType, InferVStreams,
    )
except ImportError as exc:      # Hailo 미장착 / 런타임 미설치 환경
    raise ImportError(
        "hailo_platform 을 불러올 수 없습니다. Hailo 모듈이 아직 안 붙었거나 "
        "HailoRT 가 설치되지 않은 상태입니다.\n"
        "  설치 절차: drain_vision/README.md 의 '설치가 필요한 것' 항목 참고\n"
        "  하드웨어 확인:  lspci | grep -i hailo   /   hailortcli fw-control identify\n"
        "  카메라만 띄우려면: ros2 launch drain_vision camera.launch.py\n"
        f"  (원인: {exc})"
    ) from exc

#  model_path 파라미터가 비었을 때만 쓰이는 폴백이다.
#  ★ [2026-08-29] v2 교체와 함께 여기도 바꿨다 ★
#     detector_params.yaml 만 고치면 파라미터를 안 넘기는 실행 경로
#     (`ros2 run drain_vision drain_detector`)가 조용히 옛 모델을 문다.
#     "현재 모델"의 답이 두 개가 되지 않도록 yaml 과 같은 값으로 맞춰 둔다.
DEFAULT_MODEL = 'yolov11s_v2.hef'

#  이 패키지의 .hef 는 1클래스(drain) 모델이다.
#  예전에는 COCO 80종 폴백이 있었지만 지웠다 — 1클래스 모델에 80종 이름표가 붙어 있으면
#  class_names 를 비우는 순간 cls_id=0 이 "person" 으로 찍혀 오해만 만든다.
#  모델을 다중 클래스로 바꾸면 class_names 파라미터에 콤마로 나열할 것.
DEFAULT_CLASSES = ['drain']


def _parse_classes(csv: str, fallback: list = DEFAULT_CLASSES) -> list:
    names = [s.strip() for s in (csv or "").split(",") if s.strip()]
    return names if names else list(fallback)


def _resolve_model_path(model_path: str) -> str:
    """빈 값/파일명/상대경로는 패키지 share/models 기준으로 해석한다.

    절대경로가 주어지면 그대로 사용하므로 기존 설정과도 호환된다.
    """
    models_dir = os.path.join(
        get_package_share_directory('drain_vision'), 'models')
    if not model_path:
        return os.path.join(models_dir, DEFAULT_MODEL)
    if os.path.isabs(model_path):
        return model_path
    return os.path.join(models_dir, model_path)


class DrainDetector(Node):
    def __init__(self):
        super().__init__('drain_detector')

        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', 0.3)
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)
        self.declare_parameter('class_names', '')
        #  [2026-08-22 추가] 탐지 좌표 토픽.
        #  종전에는 좌표를 계산해놓고 **그림만 그리고 버렸다.** 서보잉이 쓸 입력이 없었다.
        self.declare_parameter('detection_topic', '/detections/drains')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('detection_image_topic', '/detections/image')

        model_path = _resolve_model_path(self.get_parameter('model_path').value)
        self.conf = self.get_parameter('conf_threshold').value
        self.input_w = self.get_parameter('input_width').value
        self.input_h = self.get_parameter('input_height').value
        self.classes = _parse_classes(self.get_parameter('class_names').value)

        if not model_path or not os.path.exists(model_path):
            self.get_logger().error(f'model_path not found: "{model_path}"')
            raise RuntimeError('Invalid model_path')

        # Hailo 초기화
        hef = HEF(model_path)
        self._target = VDevice()
        cfg = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
        network_groups = self._target.configure(hef, cfg)
        self._ng = network_groups[0]
        self._ng_params = self._ng.create_params()
        self._input_params  = InputVStreamParams.make(self._ng, format_type=FormatType.UINT8)
        self._output_params = OutputVStreamParams.make(self._ng, format_type=FormatType.FLOAT32)
        self._input_name  = hef.get_input_vstream_infos()[0].name
        self._output_name = hef.get_output_vstream_infos()[0].name

        # 추론 스트림은 노드 수명 동안 1회만 생성한다.
        # (프레임마다 InferVStreams/activate 를 재생성하면 스트림 셋업 비용이
        #  매 프레임 추론 시간에 그대로 더해진다.)
        self._stack = ExitStack()
        activation = self._ng.activate(self._ng_params)
        if activation is not None:      # 스케줄러 사용 시 None 반환
            self._stack.enter_context(activation)
        self._pipeline = self._stack.enter_context(
            InferVStreams(self._ng, self._input_params, self._output_params)
        )
        self.get_logger().info(f'Hailo model loaded: {model_path}')

        image_topic = self.get_parameter('image_topic').value
        det_topic = self.get_parameter('detection_image_topic').value

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.pub = self.create_publisher(Image, det_topic, 10)
        #  좌표 토픽. 이미지와 달리 **탐지가 없어도 빈 배열로 매 프레임 발행**한다.
        #  그래야 소비자가 "탐지 없음"과 "노드 죽음"을 구별할 수 있다.
        self.det_pub = self.create_publisher(
            DrainDetectionArray, self.get_parameter('detection_topic').value, 10)
        self.get_logger().info(
            f'DrainDetector ready (conf={self.conf}, {image_topic} -> {det_topic} '
            f"+ {self.get_parameter('detection_topic').value})")

    # ------------------------------------------------------------------
    def _letterbox(self, frame):
        """임의 해상도 → input_w × input_h (비율 유지, 패딩)"""
        h, w = frame.shape[:2]
        scale = min(self.input_w / w, self.input_h / h)
        new_w, new_h = int(w * scale), int(h * scale)
        resized = cv2.resize(frame, (new_w, new_h))
        canvas = np.full((self.input_h, self.input_w, 3), 114, dtype=np.uint8)
        pad_x = (self.input_w - new_w) // 2
        pad_y = (self.input_h - new_h) // 2
        canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
        return canvas, scale, pad_x, pad_y

    def _preprocess(self, frame):
        """BGR → RGB letterbox → UINT8 NHWC (Hailo 입력 포맷)"""
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        letterboxed, scale, pad_x, pad_y = self._letterbox(rgb)
        self._last_scale = scale
        self._last_pad = (pad_x, pad_y)
        return np.expand_dims(letterboxed, axis=0)  # (1, H, W, 3)

    def _postprocess(self, raw, orig_h, orig_w):
        """Hailo NMS 출력 파싱: list[batch][class] → ndarray(N,5) [y1,x1,y2,x2,score] 정규화"""
        scale = self._last_scale
        pad_x, pad_y = self._last_pad
        detections = []
        per_class = raw[0]  # batch 0
        for cls_id, boxes in enumerate(per_class):
            if len(boxes) == 0:
                continue
            for box in boxes:
                score = float(box[4])
                if score < self.conf:
                    continue
                # 정규화 좌표 → 640px 좌표 → letterbox 역변환 → 원본 좌표
                y1 = int((float(box[0]) * self.input_h - pad_y) / scale)
                x1 = int((float(box[1]) * self.input_w - pad_x) / scale)
                y2 = int((float(box[2]) * self.input_h - pad_y) / scale)
                x2 = int((float(box[3]) * self.input_w - pad_x) / scale)
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(orig_w, x2), min(orig_h, y2)
                label = self.classes[cls_id] if cls_id < len(self.classes) else str(cls_id)
                detections.append((x1, y1, x2, y2, score, label))
        return detections

    # ------------------------------------------------------------------
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        input_batch = self._preprocess(frame)

        results = self._pipeline.infer({self._input_name: input_batch})

        detections = self._postprocess(results[self._output_name], frame.shape[0], frame.shape[1])

        for x1, y1, x2, y2, score, label in detections:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 0), 2)
            cv2.putText(frame, f'{label} {score:.2f}', (x1, max(y1 - 6, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 0), 2)

        out_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)

        self._publish_detections(msg.header, detections, frame.shape[0], frame.shape[1])

    def _publish_detections(self, header, detections, h, w):
        """탐지 좌표를 발행한다. 서보잉 노드가 쓰는 입력이다.

        정규화 좌표를 여기서 계산해 넣는 이유:
          소비자마다 픽셀 산수를 반복하면 영상 해상도가 바뀔 때 전부 고쳐야 한다.
          cx_norm 은 그대로 조향 오차로 쓸 수 있는 형태(-1.0 ~ +1.0)로 준다.
        """
        out = DrainDetectionArray()
        out.header = header          # 원본 영상 stamp 를 그대로 물려준다 (신선도 판단용)
        out.image_width = int(w)
        out.image_height = int(h)
        for x1, y1, x2, y2, score, label in detections:
            d = DrainDetection()
            d.label = str(label)
            d.score = float(score)
            d.x1, d.y1, d.x2, d.y2 = int(x1), int(y1), int(x2), int(y2)
            #  중심을 [-1, +1] 로. 화면 중앙이 0 이라 부호가 그대로 좌/우가 된다.
            d.cx_norm = float(((x1 + x2) * 0.5) / w * 2.0 - 1.0)
            d.cy_norm = float(((y1 + y2) * 0.5) / h * 2.0 - 1.0)
            #  면적비는 거리 '대용'일 뿐이다. 절대 거리로 쓰지 말 것 (TOF 가 정답)
            d.area_ratio = float(max(0, x2 - x1) * max(0, y2 - y1)) / float(w * h)
            out.detections.append(d)
        self.det_pub.publish(out)

    def shutdown_hailo(self):
        """추론 스트림과 네트워크 활성화를 역순으로 정리한다."""
        stack = getattr(self, '_stack', None)
        if stack is not None:
            stack.close()
            self._stack = None


def main():
    rclpy.init()
    node = DrainDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hailo()
        node.destroy_node()
        if rclpy.ok():          # SIGTERM 등으로 이미 shutdown 된 경우 중복 호출 방지
            rclpy.shutdown()


if __name__ == '__main__':
    main()
