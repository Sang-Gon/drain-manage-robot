import sys
import os
import site

user_site = site.getusersitepackages()
if user_site not in sys.path:
    sys.path.insert(0, user_site)

from contextlib import ExitStack

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np

from hailo_platform import (
    VDevice, HEF, ConfigureParams, HailoStreamInterface,
    InputVStreamParams, OutputVStreamParams, FormatType, InferVStreams,
)

DEFAULT_MODEL = 'yolov11s_drain.hef'

COCO_CLASSES = [
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
    "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
    "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
    "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair",
    "couch","potted plant","bed","dining table","toilet","tv","laptop","mouse",
    "remote","keyboard","cell phone","microwave","oven","toaster","sink","refrigerator",
    "book","clock","vase","scissors","teddy bear","hair drier","toothbrush",
]

def _parse_classes(csv: str, fallback: list) -> list:
    names = [s.strip() for s in csv.split(",") if s.strip()]
    return names if names else fallback


def _resolve_model_path(model_path: str) -> str:
    """빈 값/파일명/상대경로는 패키지 share/models 기준으로 해석한다.

    절대경로가 주어지면 그대로 사용하므로 기존 설정과도 호환된다.
    """
    models_dir = os.path.join(
        get_package_share_directory('my_opencv_tutorials'), 'models')
    if not model_path:
        return os.path.join(models_dir, DEFAULT_MODEL)
    if os.path.isabs(model_path):
        return model_path
    return os.path.join(models_dir, model_path)


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', 0.3)
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)
        self.declare_parameter('class_names', '')

        model_path = _resolve_model_path(self.get_parameter('model_path').value)
        self.conf = self.get_parameter('conf_threshold').value
        self.input_w = self.get_parameter('input_width').value
        self.input_h = self.get_parameter('input_height').value
        self.classes = _parse_classes(
            self.get_parameter('class_names').value, COCO_CLASSES
        )

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

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Image, '/detections/image', 10)
        self.get_logger().info(f'YoloDetector ready (conf={self.conf})')

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

    def shutdown_hailo(self):
        """추론 스트림과 네트워크 활성화를 역순으로 정리한다."""
        stack = getattr(self, '_stack', None)
        if stack is not None:
            stack.close()
            self._stack = None


def main():
    rclpy.init()
    node = YoloDetector()
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
