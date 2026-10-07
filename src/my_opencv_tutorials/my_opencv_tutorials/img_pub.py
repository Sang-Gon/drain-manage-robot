import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2

class ImgPublisher(Node):
    def __init__(self):
        super().__init__('img_publisher')
        self.publisher = self.create_publisher(Image, '/image_raw', 10)

        self.declare_parameter('width', 640)
        self.width = self.get_parameter('width').value
        self.declare_parameter('height', 480)
        self.height = self.get_parameter('height').value
        self.declare_parameter('camera_device', 0)
        self.camera_device = self.get_parameter('camera_device').value
        self.declare_parameter('frame_rate', 10)
        self.frame_rate = self.get_parameter('frame_rate').value

        self.cap = cv2.VideoCapture(self.camera_device)
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
        self.get_logger().info(
            "Camera negotiated : %dx%d @ %.1ffps" % (actual_w, actual_h, actual_fps))
        if (actual_w, actual_h) != (self.width, self.height):
            self.get_logger().warn(
                "웹캠이 요청 해상도를 지원하지 않아 CPU resize로 대체합니다.")

    def time_callback(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn("프레임 획득 실패 — 이번 주기를 건너뜁니다.",
                                   throttle_duration_sec=2.0)
            return
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        img = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = 'camera'
        self.publisher.publish(img)

def main() :
    rclpy.init()
    node = ImgPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        if rclpy.ok():          # SIGTERM 등으로 이미 shutdown 된 경우 중복 호출 방지
            rclpy.shutdown()

if __name__ == '__main__' :
    main()