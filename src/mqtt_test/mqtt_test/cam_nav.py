"""
cam_nav — Camera-to-Map navigation node

Reads a live camera feed, transforms mouse-clicked pixels to map-frame
coordinates using a pre-calibrated homography (from cam_caribation.py),
then publishes a PoseStamped to /goal_pose so Nav2 can drive the robot there.

Calibration: update image_points and map_points to match your camera setup.
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# ── Calibration (from cam_caribation.py) ─────────────────────────────────────
# Pixel corners of the observable floor area in the camera frame
image_points = np.array(
    [
        [262, 432],   # Anchor 1
        [972, 322],   # Anchor 2
        [1278, 348],  # Anchor 3
        [1278, 718],  # Anchor 4
    ],
    dtype=np.float32,
)

# Corresponding real-world coordinates in the map frame (metres)
map_points = np.array(
    [
        [0.0, 2.0],  # Far-left
        [2.0, 2.0],  # Far-right
        [0.0, 0.0],  # Near-left
        [2.0, 0.0],  # Near-right
    ],
    dtype=np.float32,
)

H, _ = cv2.findHomography(image_points, map_points)


class CamNavNode(Node):
    def __init__(self):
        super().__init__("cam_nav")
        self._pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.get_logger().info("CamNav ready — click the feed to navigate")

    def send_goal(self, x: float, y: float) -> None:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self._pub.publish(msg)
        self.get_logger().info(f"Goal sent  x={x:.3f} m  y={y:.3f} m")


def main(args=None):
    rclpy.init(args=args)
    node = CamNavNode()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        node.get_logger().error("Cannot open camera device 0")
        node.destroy_node()
        rclpy.shutdown()
        return

    last_goal_px = None

    def on_click(event, x, y, flags, param):
        nonlocal last_goal_px
        if event == cv2.EVENT_LBUTTONDOWN:
            px = np.array([[[float(x), float(y)]]], dtype=np.float32)
            m = cv2.perspectiveTransform(px, H)
            gx, gy = float(m[0][0][0]), float(m[0][0][1])
            node.get_logger().info(f"Pixel ({x},{y})  →  map ({gx:.3f}, {gy:.3f}) m")
            node.send_goal(gx, gy)
            last_goal_px = (x, y)

    cv2.namedWindow("Camera Nav")
    cv2.setMouseCallback("Camera Nav", on_click)

    _read_fails = 0

    while rclpy.ok():
        try:
            ret, frame = cap.read()
        except Exception as exc:
            node.get_logger().error(f"cap.read() raised: {exc}")
            break

        if not ret or frame is None:
            _read_fails += 1
            if _read_fails > 300:
                node.get_logger().error("Camera unavailable. Exiting.")
                break
            time.sleep(0.03)
            cv2.waitKey(30)
            continue

        _read_fails = 0

        cv2.putText(frame, "Click to set nav goal  |  Q = quit",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
        if last_goal_px:
            cv2.circle(frame, last_goal_px, 10, (0, 0, 255), -1)
            cv2.putText(frame, f"Goal: {last_goal_px}",
                        (last_goal_px[0] + 12, last_goal_px[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.imshow("Camera Nav", frame)

        rclpy.spin_once(node, timeout_sec=0.01)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
