"""
click_to_goal_node.py
=====================
ROS 2 node — คลิกบน webcam แล้วส่ง NavigateToPose goal ไปหุ่นยนต์ทันที

Pipeline:
  pixel กล้อง (u,v)
    → [Homography H]
    → pixel แผนที่ (col, row)
    → [map.yaml: resolution + origin]
    → world coordinate (x_w, y_w) เมตร  ← ระบบเดียวกับ /map frame ของ SLAM
    → geometry_msgs/PoseStamped → nav2_msgs/action/NavigateToPose

ต้องการ:
  - homography.json (ได้จาก calibrate_camera_to_map.py)
  - ROS 2 Humble/Iron พร้อม nav2_msgs
  - pip install opencv-python numpy pyyaml

วิธีรัน:
  ros2 run <your_pkg> click_to_goal_node
  หรือ:
  python click_to_goal_node.py
"""

import os
import cv2
import numpy as np
import json
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Header

HOMOGRAPHY_FILE = "/home/parichu/ros2_ws/mqtt_test/src/mqtt_test/config/homography.json"
CAMERA_SOURCE = 2
MAP_FRAME = "map"  # tf frame ของแผนที่ SLAM
DEFAULT_YAW = 0.0  # หันหน้าไปทิศไหน (radians) 0 = +x
WINDOW_TITLE = "Click to Navigate"


# ============================================================
# ฟังก์ชันแปลงพิกัด
# ============================================================
def pixel_to_world(col: float, row: float, cfg: dict) -> tuple[float, float]:
    """
    แปลง pixel แผนที่ (col, row) → world (x_w, y_w) เมตร

    map.yaml กำหนดว่า:
      - origin คือตำแหน่ง world ของมุม bottom-left ของแผนที่
      - แกน y ของ pixel นับลงล่าง แต่ world นับขึ้นบน
      ดังนั้น:
        x_w = origin[0] + col  * resolution
        y_w = origin[1] + (map_height - row) * resolution
    """
    res = cfg["resolution"]
    origin = cfg["origin"]  # [x, y, theta]
    h_px = cfg["height_px"]  # จำนวน pixel แนวตั้งของแผนที่
    x_w = origin[0] + col * res
    y_w = origin[1] + (h_px - row) * res
    return x_w, y_w


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """yaw (rad) → quaternion (x,y,z,w)"""
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


# ============================================================
# ROS 2 Node
# ============================================================
class ClickToGoalNode(Node):
    def __init__(self, H, map_cfg):
        super().__init__("click_to_goal_node")
        self.H = H
        self.map_cfg = map_cfg
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._current_goal_handle = None
        self.get_logger().info("Waiting for Nav2 navigate_to_pose action server...")
        self._nav_client.wait_for_server()
        self.get_logger().info("Nav2 ready.")

    def send_goal(self, x_w: float, y_w: float, yaw: float = DEFAULT_YAW):
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

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

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f"Sending goal → world ({x_w:.3f}, {y_w:.3f}) m  yaw={math.degrees(yaw):.1f}°"
        )

        # ยกเลิก goal เก่าถ้ามี
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()

        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_cb,
        )
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            return
        self._current_goal_handle = handle
        self.get_logger().info("Goal accepted — robot navigating...")
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f"Navigation result: {result}")

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        d = fb.distance_remaining
        self.get_logger().info(
            f"  Distance remaining: {d:.2f} m", throttle_duration_sec=1.0
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
node_ref: ClickToGoalNode | None = None  # set ใน main()


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
    """OpenCV window loop (รันใน thread แยก)"""
    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WINDOW_TITLE, mouse_callback)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w = frame.shape[:2]
        cv2.rectangle(frame, (0, h - 22), (w, h), (0, 0, 0), -1)
        cv2.putText(
            frame,
            "คลิกบนกล้องเพื่อส่ง goal  |  Q = ออก",
            (8, h - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (200, 200, 200),
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

    # โหลด homography.json
    if not os.path.exists(HOMOGRAPHY_FILE):
        print(f"[!] ไม่พบ {HOMOGRAPHY_FILE}")
        print("    รัน calibrate_camera_to_map.py ก่อน")
        return

    with open(HOMOGRAPHY_FILE) as f:
        cal = json.load(f)

    H = np.float32(cal["H"])
    my_cfg = cal["map_yaml"].copy()

    # เพิ่ม height_px — อ่านจาก pgm ถ้ายังไม่มีใน json
    if "height_px" not in my_cfg:
        yaml_dir = os.path.dirname(HOMOGRAPHY_FILE)
        pgm_path = os.path.join(yaml_dir, my_cfg["image"])
        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        my_cfg["height_px"] = img.shape[0]

    # เปิดกล้อง
    cap = cv2.VideoCapture(CAMERA_SOURCE)
    if not cap.isOpened():
        print(f"[!] เปิดกล้องไม่ได้: {CAMERA_SOURCE}")
        return

    # เริ่ม ROS 2
    rclpy.init()
    node = ClickToGoalNode(H, my_cfg)
    node_ref = node

    # รัน ROS 2 spin ใน thread แยก
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # UI loop ใน main thread (OpenCV ต้องการ main thread บน macOS/Linux)
    ui_loop(cap)

    # cleanup
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join(timeout=2)


if __name__ == "__main__":
    import os

    main()
