"""
click_point.py — External-camera click-to-navigate with live map overlay

Two windows:
  "Robot Eye"  — rapoo external camera (device 2, 1920×1080)
                 Double-click any floor position → robot navigates there
  "Map View"   — mapuse6.pgm with overlays:
                   ● Green circle + arrow  = live robot position (/odom)
                   ● Red   circle          = last sent navigation goal
                   ◆ Cyan  diamonds        = calibration anchor points

── Calibration (CALIBRATE_MODE = True) ──────────────────────────────────────
Step 1 — Robot Eye:  double-click 4 floor anchors  (camera pixel → image_points)
Step 2 — Map View:   double-click the SAME 4 spots  (map pixel → map_points m)

Both sets come from real clicks; no made-up numbers needed.
Printed values at the end → paste into image_points / map_points below.

Key bindings:
  Double-click  — navigate (normal mode) / pick calibration point (calib mode)
  C             — clear goal marker
  Q / Esc       — quit
"""

import math
import os
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

# ── Map metadata (from mapuse6.yaml) ─────────────────────────────────────────
_pkg           = get_package_share_directory("mqtt_test")
MAP_IMAGE_PATH = os.path.join(_pkg, "config", "mapuse6.pgm")
MAP_ORIGIN_X   = -5.53    # metres
MAP_ORIGIN_Y   = -5.94    # metres
MAP_RESOLUTION = 0.03     # metres / pixel

_MAP_IMG = cv2.imread(MAP_IMAGE_PATH, cv2.IMREAD_GRAYSCALE)
if _MAP_IMG is None:
    raise FileNotFoundError(f"Cannot load map: {MAP_IMAGE_PATH}")
MAP_H, MAP_W = _MAP_IMG.shape   # 461 × 825

# ── Calibration data ──────────────────────────────────────────────────────────
# Pixels clicked on the rapoo camera (device 2 @ 1920×1080).
# Run with CALIBRATE_MODE = True to re-pick these live.
image_points = np.array(
    [
        [262,  432],
        [972,  322],
        [1278, 348],
        [1278, 718],
    ],
    dtype=np.float32,
)

# Matching ROS2 map-frame coordinates (metres).
# Run with CALIBRATE_MODE = True to pick these by clicking mapuse6.pgm.
map_points = np.array(
    [
        [0.0, 2.0],
        [2.0, 2.0],
        [0.0, 0.0],
        [2.0, 0.0],
    ],
    dtype=np.float32,
)

# ── Set True to recalibrate both image_points and map_points live ─────────────
CALIBRATE_MODE = False

H, _mask = cv2.findHomography(image_points, map_points)


# ── Coordinate helpers ────────────────────────────────────────────────────────

def map_m_to_px(x_m: float, y_m: float) -> tuple:
    """Map-frame metres → map image (col, row). Y-axis flipped."""
    col = int((x_m - MAP_ORIGIN_X) / MAP_RESOLUTION)
    row = int(MAP_H - (y_m - MAP_ORIGIN_Y) / MAP_RESOLUTION)
    return col, row


def map_px_to_m(col: int, row: int) -> tuple:
    """Map image (col, row) → map-frame metres. Inverse of map_m_to_px."""
    x_m = col * MAP_RESOLUTION + MAP_ORIGIN_X
    y_m = (MAP_H - row) * MAP_RESOLUTION + MAP_ORIGIN_Y
    return x_m, y_m


def cam_px_to_map(u: int, v: int) -> tuple:
    """Camera pixel → map-frame metres via homography H."""
    pt = np.array([[[float(u), float(v)]]], dtype=np.float32)
    m  = cv2.perspectiveTransform(pt, H)
    return float(m[0][0][0]), float(m[0][0][1])


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class ClickNavNode(Node):
    def __init__(self):
        super().__init__("click_nav")
        self._pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_yaw = 0.0
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.get_logger().info("ClickNav ready — double-click camera to navigate")

    def _odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def send_goal(self, x: float, y: float):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        self._pub.publish(msg)
        self.get_logger().info(f"Goal → x={x:.3f} m  y={y:.3f} m")


# ── Main ──────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ClickNavNode()

    cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        node.get_logger().error("Cannot open external camera on /dev/video2")
        node.destroy_node()
        rclpy.shutdown()
        return

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    node.get_logger().info(f"Camera resolution: {actual_w}×{actual_h}")

    map_bgr = cv2.cvtColor(_MAP_IMG, cv2.COLOR_GRAY2BGR)

    # ── Shared navigation state ───────────────────────────────────────────────
    goal_cam_px = None
    goal_map_px = None

    # ── Calibration state machine ─────────────────────────────────────────────
    # stage: "camera" → collect 4 camera clicks
    #        "map"    → collect 4 map clicks
    #        "done"   → print result
    calib = {
        "stage":    "camera",
        "cam_pts":  [],   # list of (u, v) camera pixels
        "map_pts":  [],   # list of (x_m, y_m) map-frame metres
    }

    def _print_calib_result():
        print("\n" + "="*60)
        print("CALIBRATION COMPLETE — paste into click_point.py:")
        print("="*60)
        print("\nimage_points = np.array([")
        for u, v in calib["cam_pts"]:
            print(f"    [{u}, {v}],")
        print("], dtype=np.float32)\n")
        print("map_points = np.array([")
        for x, y in calib["map_pts"]:
            print(f"    [{x:.4f}, {y:.4f}],")
        print("], dtype=np.float32)")
        print("="*60 + "\n")

    # ── Mouse callback: Robot Eye (camera window) ─────────────────────────────
    def on_camera_mouse(event, u, v, flags, param):
        nonlocal goal_cam_px, goal_map_px
        if event != cv2.EVENT_LBUTTONDBLCLK:
            return

        if CALIBRATE_MODE:
            if calib["stage"] == "camera" and len(calib["cam_pts"]) < 4:
                calib["cam_pts"].append((u, v))
                n = len(calib["cam_pts"])
                print(f"  [Camera] Anchor {n}/4: pixel ({u}, {v})")
                if n == 4:
                    calib["stage"] = "map"
                    print("\n  Camera done. Now double-click the SAME 4 points")
                    print("  on the Map View window (same order).\n")
        else:
            gx, gy      = cam_px_to_map(u, v)
            goal_cam_px = (u, v)
            goal_map_px = map_m_to_px(gx, gy)
            node.get_logger().info(f"Camera ({u},{v}) → map ({gx:.3f}, {gy:.3f}) m")
            node.send_goal(gx, gy)

    # ── Mouse callback: Map View (map window) ─────────────────────────────────
    def on_map_mouse(event, col, row, flags, param):
        if event != cv2.EVENT_LBUTTONDBLCLK:
            return

        if CALIBRATE_MODE:
            if calib["stage"] == "map" and len(calib["map_pts"]) < 4:
                x_m, y_m = map_px_to_m(col, row)
                calib["map_pts"].append((x_m, y_m))
                n = len(calib["map_pts"])
                print(f"  [Map]    Anchor {n}/4: pixel ({col},{row}) → ({x_m:.4f}, {y_m:.4f}) m")
                if n == 4:
                    calib["stage"] = "done"
                    _print_calib_result()

    cv2.namedWindow("Robot Eye")
    cv2.setMouseCallback("Robot Eye", on_camera_mouse)
    cv2.namedWindow("Map View")
    cv2.setMouseCallback("Map View", on_map_mouse)

    # Warm up: discard first few frames (V4L2 buffers stale data)
    node.get_logger().info("Warming up camera...")
    for _ in range(3):
        cap.read()

    _read_fails = 0

    while rclpy.ok():
        # ── Grab camera frame ─────────────────────────────────────────────────
        try:
            ret, frame = cap.read()
        except Exception as exc:
            node.get_logger().error(f"cap.read() raised: {exc}")
            break

        if not ret or frame is None:
            _read_fails += 1
            if _read_fails % 30 == 1:
                node.get_logger().warn(
                    f"Camera read failed (attempt {_read_fails}). "
                    "Check /dev/video2."
                )
            if _read_fails > 300:
                node.get_logger().error("Camera unavailable. Exiting.")
                break
            # Use time.sleep here — rclpy.spin_once can fail before the
            # first successful frame if the ROS context hasn't settled yet
            time.sleep(0.03)
            cv2.waitKey(30)
            continue
        _read_fails = 0

        # ── Robot Eye window ──────────────────────────────────────────────────
        if CALIBRATE_MODE:
            stage = calib["stage"]
            n_cam = len(calib["cam_pts"])
            n_map = len(calib["map_pts"])

            if stage == "camera":
                msg = f"STEP 1/2 — Double-click anchor {n_cam+1}/4 on the FLOOR (camera)"
                color = (0, 140, 255)
            elif stage == "map":
                msg = f"STEP 2/2 — Now double-click anchor {n_map+1}/4 on the MAP window"
                color = (0, 220, 100)
            else:
                msg = "Calibration done — see terminal. Set CALIBRATE_MODE=False"
                color = (0, 255, 0)

            cv2.putText(frame, msg, (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

            for i, (u, v) in enumerate(calib["cam_pts"]):
                cv2.circle(frame, (u, v), 9, (0, 140, 255), -1)
                cv2.putText(frame, str(i + 1), (u + 12, v - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 140, 255), 2)
        else:
            cv2.putText(frame, "Double-click to navigate  |  C=clear  Q=quit",
                        (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 0), 2)

            for pt in image_points.astype(int):
                cv2.drawMarker(frame, tuple(pt), (0, 220, 255),
                               cv2.MARKER_CROSS, 24, 2)

            if goal_cam_px:
                cv2.circle(frame, goal_cam_px, 11, (0, 0, 255), -1)
                cv2.putText(frame, f"Goal {goal_cam_px}",
                            (goal_cam_px[0] + 14, goal_cam_px[1] - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1)

        cv2.imshow("Robot Eye", frame)

        # ── Map View window ───────────────────────────────────────────────────
        m = map_bgr.copy()

        if CALIBRATE_MODE:
            stage = calib["stage"]
            n_map = len(calib["map_pts"])

            if stage == "map":
                hint = f"Double-click anchor {n_map+1}/4 here"
                cv2.putText(m, hint, (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 100), 1)

            # Show already-collected map calibration points
            for i, (x_m, y_m) in enumerate(calib["map_pts"]):
                mpx = map_m_to_px(x_m, y_m)
                cv2.circle(m, mpx, 8, (0, 140, 255), -1)
                cv2.putText(m, str(i + 1), (mpx[0] + 10, mpx[1] - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 140, 255), 2)
        else:
            # Calibration reference anchors (cyan diamonds)
            for pt in map_points:
                mpx = map_m_to_px(pt[0], pt[1])
                cv2.drawMarker(m, mpx, (0, 200, 255),
                               cv2.MARKER_DIAMOND, 18, 2)

            # Navigation goal
            if goal_map_px:
                cv2.circle(m, goal_map_px, 9, (0, 0, 255), -1)
                cv2.drawMarker(m, goal_map_px, (0, 0, 255),
                               cv2.MARKER_CROSS, 22, 2)

            # Robot position
            rx, ry = map_m_to_px(node.robot_x, node.robot_y)
            cv2.circle(m, (rx, ry), 8, (0, 210, 0), -1)
            ax = int(rx + 22 * math.cos(node.robot_yaw))
            ay = int(ry - 22 * math.sin(node.robot_yaw))
            cv2.arrowedLine(m, (rx, ry), (ax, ay), (0, 255, 0), 2, tipLength=0.35)

            # Legend
            lx, ly = 10, MAP_H - 60
            cv2.circle(m, (lx+6, ly),    5, (0, 210, 0),  -1)
            cv2.putText(m, "Robot", (lx+15, ly+4),   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,210,0),  1)
            cv2.circle(m, (lx+6, ly+16), 5, (0, 0, 255),  -1)
            cv2.putText(m, "Goal",  (lx+15, ly+20),  cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255),  1)
            cv2.drawMarker(m, (lx+6, ly+32), (0, 200, 255), cv2.MARKER_DIAMOND, 10, 1)
            cv2.putText(m, "Calib", (lx+15, ly+36),  cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,200,255),1)

        cv2.imshow("Map View", m)

        rclpy.spin_once(node, timeout_sec=0.01)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key == ord("c"):
            goal_cam_px = None
            goal_map_px = None

    cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
