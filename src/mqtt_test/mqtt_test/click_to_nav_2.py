"""
click_to_goal_node.py (Updated: ROS Topic + MQTT Auth & Payload)
================================================================
ROS 2 node — คลิกบน webcam แล้วส่งพิกัดเป้าหมาย (x, y)
ไปที่ ROS Topic `/goal_xy` และส่งผ่าน MQTT ทันที (พร้อม Auth)

ต้องการ:
  - homography.json (ได้จาก calibrate_camera_to_map.py)
  - ROS 2 Humble/Iron
  - pip install opencv-python numpy pyyaml paho-mqtt

วิธีรัน:
  python click_to_goal_node.py
"""

import os
import cv2
import numpy as np
import json
import math
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import paho.mqtt.client as mqtt

# ============================================================
# ตั้งค่าคอนฟิกต่างๆ
# ============================================================
HOMOGRAPHY_FILE = "/home/parichu/ros2_ws/mqtt_test/src/mqtt_test/config/homography.json"
CAMERA_SOURCE = 0
MAP_FRAME = "map"  # tf frame ของแผนที่ SLAM
DEFAULT_YAW = 0.0  # หันหน้าไปทิศไหน (radians) 0 = +x
WINDOW_TITLE = "Click to Navigate (ROS & MQTT)"

# --- MQTT Config ---
MQTT_BROKER = "172.20.10.6"  # เปลี่ยนเป็น IP ของ Broker ที่คุณใช้งาน
MQTT_PORT = 1883
MQTT_TOPIC = "robot/nav_goal"
MQTT_USERNAME = "parichu"
MQTT_PASSWORD = "1122"
# -------------------


def pixel_to_world(col: float, row: float, cfg: dict) -> tuple[float, float]:
    """แปลง pixel แผนที่ (col, row) → world (x_w, y_w) เมตร"""
    res = cfg["resolution"]
    origin = cfg["origin"]
    h_px = cfg["height_px"]
    x_w = origin[0] + col * res
    y_w = origin[1] + (h_px - row) * res
    return x_w, y_w


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """yaw (rad) → quaternion (x,y,z,w)"""
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


# ============================================================
# ROS 2 Node + MQTT Publisher
# ============================================================
class ClickToGoalNode(Node):
    def __init__(self, H, map_cfg):
        super().__init__("click_to_goal_node")
        self.H = H
        self.map_cfg = map_cfg

        # 1. Setup ROS 2 Publisher
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_xy", 10)

        # 2. Setup MQTT Client
        self.mqtt_connected = False
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_disconnect = self._on_disconnect

        try:
            self.mqtt_client.connect_async(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()  # ให้ MQTT รันใน background
            self.get_logger().info(
                f"กำลังเชื่อมต่อ MQTT Broker ({MQTT_BROKER}:{MQTT_PORT})..."
            )
        except Exception as e:
            self.get_logger().warn(f"ไม่สามารถเริ่มการเชื่อมต่อ MQTT ได้: {e}")
            self.mqtt_client = None

        self.get_logger().info("Node พร้อมทำงาน ส่งข้อมูลไปที่ topic '/goal_xy' และ MQTT")

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.mqtt_connected = True
            self.get_logger().info("MQTT เชื่อมต่อสำเร็จแล้ว!")
        else:
            self.get_logger().error(f"เชื่อมต่อ MQTT ไม่สำเร็จ (Code: {rc})")

    def _on_disconnect(self, client, userdata, rc):
        self.mqtt_connected = False
        self.get_logger().warn(f"MQTT ขาดการเชื่อมต่อ (Return code: {rc})")

    def send_goal(self, x_w: float, y_w: float, yaw: float = DEFAULT_YAW):
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        # --- ส่งผ่าน ROS 2 Topic ---
        pose = PoseStamped()
        pose.header = Header(frame_id=MAP_FRAME)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x_w
        pose.pose.position.y = y_w
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.goal_pub.publish(pose)

        # --- ส่งผ่าน MQTT ---
        if self.mqtt_client is not None and self.mqtt_connected:
            payload = {
                "Position x": float(round(x_w, 3)),
                "Position y": float(round(y_w, 3)),
                "Time Stamped": int(time.time()),
            }
            try:
                self.mqtt_client.publish(MQTT_TOPIC, json.dumps(payload))
                mqtt_status = "และ MQTT "
            except Exception as e:
                self.get_logger().error(f"ส่ง MQTT ไม่สำเร็จ: {e}")
                mqtt_status = "(MQTT Error) "
        else:
            mqtt_status = "(ไม่ได้เชื่อมต่อ MQTT) "

        self.get_logger().info(
            f"ส่งเป้าหมาย {mqtt_status}→ world ({x_w:.3f}, {y_w:.3f}) m"
        )

    def transform_click(self, u: int, v: int) -> tuple[float, float] | None:
        """pixel กล้อง → world (x_w, y_w)"""
        if self.H is None:
            return None
        pt = np.float32([[[u, v]]])
        result = cv2.perspectiveTransform(pt, self.H)[0][0]
        col, row = float(result[0]), float(result[1])
        return pixel_to_world(col, row, self.map_cfg)


# ============================================================
# OpenCV UI thread
# ============================================================
node_ref: ClickToGoalNode | None = None


def mouse_callback(event, u, v, flags, _):
    if event != cv2.EVENT_LBUTTONDOWN or node_ref is None:
        return
    result = node_ref.transform_click(u, v)
    if result is None:
        print("[!] ยังไม่มี Homography")
        return
    x_w, y_w = result
    print(f"[Click] pixel ({u},{v}) → world ({x_w:.3f}, {y_w:.3f}) m")
    node_ref.send_goal(x_w, y_w)


def ui_loop(cap):
    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WINDOW_TITLE, mouse_callback)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w = frame.shape[:2]
        cv2.rectangle(frame, (0, h - 22), (w, h), (0, 0, 0), -1)

        # แสดงสถานะการเชื่อมต่อ MQTT บนจอ
        mqtt_status_text = (
            "MQTT: ON" if (node_ref and node_ref.mqtt_connected) else "MQTT: OFF"
        )
        color = (0, 255, 0) if (node_ref and node_ref.mqtt_connected) else (0, 0, 255)

        cv2.putText(
            frame,
            f"Click to send goal | Q = Exit | {mqtt_status_text}",
            (8, h - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
        )

        cv2.imshow(WINDOW_TITLE, frame)
        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


# ============================================================
# Main
# ============================================================
def main():
    global node_ref

    if not os.path.exists(HOMOGRAPHY_FILE):
        print(f"[!] ไม่พบ {HOMOGRAPHY_FILE}\n    รัน calibrate_camera_to_map.py ก่อน")
        return

    with open(HOMOGRAPHY_FILE) as f:
        cal = json.load(f)

    H = np.float32(cal["H"])
    my_cfg = cal["map_yaml"].copy()

    if "height_px" not in my_cfg:
        yaml_dir = os.path.dirname(HOMOGRAPHY_FILE)
        pgm_path = os.path.join(yaml_dir, my_cfg["image"])
        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is not None:
            my_cfg["height_px"] = img.shape[0]
        else:
            print("[!] โหลดไฟล์ภาพแผนที่ไม่ได้")
            return

    cap = cv2.VideoCapture(CAMERA_SOURCE)
    if not cap.isOpened():
        print(f"[!] เปิดกล้องไม่ได้: {CAMERA_SOURCE}")
        return

    rclpy.init()
    node = ClickToGoalNode(H, my_cfg)
    node_ref = node

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    ui_loop(cap)

    # Cleanup
    if node.mqtt_client is not None:
        node.mqtt_client.loop_stop()
        node.mqtt_client.disconnect()

    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join(timeout=2)


if __name__ == "__main__":
    main()
