#--------------------------------------------------------------------------------
# TOPIC SPEC
#
# /drain_detection (std_msgs/String, JSON)
# {
#   "drain_id": "D-003",
#   "full": true,
#   "confidence": 0.94,
#   "photo_path": "/tmp/myonge_captures/D-003_20260811.jpg"
# }
#
# /robot_status (std_msgs/String, JSON)
# {
#   "battery": 78,
#   "lat": 37.2225,
#   "lng": 127.1886
# }
#--------------------------------------------------------------------------------

import json
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import paho.mqtt.client as mqtt
import requests

#--------------------------------------------------------------------------------
# CONFIG
#--------------------------------------------------------------------------------
MQTT_HOST = "146.56.110.22"
MQTT_PORT = 1883
MQTT_USER = "myongE_01"
MQTT_PASSWORD = "0000"
ROBOT_ID = "01"

UPLOAD_URL = "https://data-myonge.yuhkm.kr/upload"
UPLOAD_API_KEY = "myongE_upload_9f3a7c2e"

#--------------------------------------------------------------------------------
# MQTT CLIENT SETUP
#--------------------------------------------------------------------------------
mqtt_client = mqtt.Client()
mqtt_client.username_pw_set(MQTT_USER, MQTT_PASSWORD)


def on_mqtt_connect(client, userdata, flags, reason_code, properties=None):
    if reason_code == 0:
        print(f"[MQTT] connected to {MQTT_HOST}:{MQTT_PORT}")
    else:
        print(f"[MQTT] connect failed code={reason_code}")


mqtt_client.on_connect = on_mqtt_connect
mqtt_client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
mqtt_client.loop_start()

#--------------------------------------------------------------------------------
# UPLOAD PHOTO (returns photo_url, or "" on failure)
#--------------------------------------------------------------------------------
def upload_photo(drain_id, image_path):
    try:
        with open(image_path, 'rb') as f:
            files = {'photo': f}
            data = {'drain_id': drain_id}
            headers = {'X-API-Key': UPLOAD_API_KEY}
            response = requests.post(UPLOAD_URL, files=files, data=data, headers=headers, timeout=10)
        response.raise_for_status()
        result = response.json()
        print(f"[UPLOAD] drain_id={drain_id} photo_url={result.get('photo_url')}")
        return result.get('photo_url', '')
    except Exception as e:
        print(f"[UPLOAD] failed drain_id={drain_id} err={e}")
        return ''

#--------------------------------------------------------------------------------
# ROS2 NODE
#--------------------------------------------------------------------------------
class ServerBridge(Node):
    def __init__(self):
        super().__init__('server_bridge')

        # topic spec agreed with vision/status team (robot_id not included;
        # this node already knows its own ROBOT_ID):
        # /drain_detection -> {drain_id, full, confidence, photo_path}
        # /robot_status    -> {battery, lat, lng}
        self.create_subscription(String, '/drain_detection', self.on_detection, 10)
        self.create_subscription(String, '/robot_status', self.on_status, 10)

        self.get_logger().info('server_bridge node started, subscribed to /drain_detection, /robot_status')

    def on_detection(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'invalid JSON on /drain_detection: {msg.data}')
            return

        drain_id = data.get('drain_id')
        is_full = data.get('full', False)
        confidence = data.get('confidence')
        photo_path = data.get('photo_path')

        if not drain_id:
            self.get_logger().error(f'drain_detection missing drain_id: {data}')
            return

        photo_url = upload_photo(drain_id, photo_path) if photo_path else ''

        report = {
            "robot_id": ROBOT_ID,
            "drain_id": drain_id,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "status": "needs_cleaning" if is_full else "normal",
            "confidence": confidence,
            "photo_url": photo_url
        }
        topic = f"myongE/{ROBOT_ID}/report"
        mqtt_client.publish(topic, json.dumps(report))
        self.get_logger().info(f'[PUBLISH][REPORT] {topic} -> {report}')

    def on_status(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'invalid JSON on /robot_status: {msg.data}')
            return

        status = {
            "lat": data.get('lat'),
            "lng": data.get('lng'),
            "battery": data.get('battery'),
            "status": "patrolling"
        }
        topic = f"myongE/{ROBOT_ID}/status"
        mqtt_client.publish(topic, json.dumps(status))
        self.get_logger().info(f'[PUBLISH][STATUS] {topic} -> {status}')


def main(args=None):
    rclpy.init(args=args)
    node = ServerBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
