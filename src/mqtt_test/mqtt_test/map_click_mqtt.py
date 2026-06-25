"""
map_click_mqtt.py — Dual-window click-to-navigate using MQTT for mapuse28

Two windows:
  1. "Map MQTT - mapuse28" — mapuse28.pgm with overlays:
      ● Green circle + arrow  = live robot position (/odom)
      ● Red   circle + cross  = last sent navigation goal
  2. "Camera MQTT" — live webcam feed with homography-based coordinate
     transformation from camera pixels to robot map-frame metres.

LEFT CLICK on either window to send a goal_pose via MQTT. Map clicks are
converted to map-frame metres using the mapuse28.yaml origin and
resolution. Camera clicks are converted using a pre-calibrated homography.
Both are published as a JSON payload to the MQTT topic.

Key bindings:
  Left click — send navigation goal (map or camera)
  C          — clear goal marker
  Q / Esc    — quit
"""

import json
import math
import os
import time

import cv2
import numpy as np
import paho.mqtt.client as mqtt

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

# ── Map metadata (from mapuse28.yaml) ─────────────────────────────────────────
_pkg = get_package_share_directory("mqtt_test")
MAP_IMAGE_PATH = os.path.join(_pkg, "config", "mapuse28.pgm")
MAP_ORIGIN_X = -3.76  # metres
MAP_ORIGIN_Y = -3.22  # metres
MAP_RESOLUTION = 0.03  # metres / pixel

_MAP_IMG = cv2.imread(MAP_IMAGE_PATH, cv2.IMREAD_GRAYSCALE)
if _MAP_IMG is None:
    raise FileNotFoundError(f"Cannot load map: {MAP_IMAGE_PATH}")
MAP_H, MAP_W = _MAP_IMG.shape

# ── Webcam-to-Robot homography (Map3 calibration) ────────────────────────────
CAM_POINTS = np.array(
    [
        [1656, 1059],
        [348, 597],
        [1008, 101],
        [1759, 150],
    ],
    dtype=np.float32,
)

WORLD_POINTS = np.array(
    [
        [6.180, 1.253],
        [-44.184, 44.958],
        [6.638, 1.490],
        [-3.901, 5.239],
    ],
    dtype=np.float32,
)

H_CAM, _ = cv2.findHomography(CAM_POINTS, WORLD_POINTS, cv2.RANSAC, 5.0)


# ── Coordinate helpers ────────────────────────────────────────────────────────


def map_m_to_px(x_m: float, y_m: float) -> tuple:
    """Map-frame metres → map image (col, row). Y-axis flipped."""
    col = int((x_m - MAP_ORIGIN_X) / MAP_RESOLUTION)
    row = int(MAP_H - 1 - (y_m - MAP_ORIGIN_Y) / MAP_RESOLUTION)
    return col, row


def map_px_to_m(col: int, row: int) -> tuple:
    """Map image (col, row) → map-frame metres. Inverse of map_m_to_px."""
    x_m = MAP_ORIGIN_X + (col * MAP_RESOLUTION)
    y_m = MAP_ORIGIN_Y + ((MAP_H - 1 - row) * MAP_RESOLUTION)
    return x_m, y_m


# ── ROS2 node ─────────────────────────────────────────────────────────────────


class MapClickMqttNode(Node):
    """ROS2 node that publishes clicked map positions as navigation goals over MQTT."""

    def __init__(self):
        super().__init__("map_click_mqtt")

        # Declare parameters
        self.declare_parameter("mqtt_broker", "172.20.10.6")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("mqtt_topic", "robot/nav_goal")
        self.declare_parameter("mqtt_username", "parichu")
        self.declare_parameter("mqtt_password", "1122")

        # Get parameters
        broker = self.get_parameter("mqtt_broker").get_parameter_value().string_value
        port = self.get_parameter("mqtt_port").get_parameter_value().integer_value
        self._topic = (
            self.get_parameter("mqtt_topic").get_parameter_value().string_value
        )
        username = (
            self.get_parameter("mqtt_username").get_parameter_value().string_value
        )
        password = (
            self.get_parameter("mqtt_password").get_parameter_value().string_value
        )

        self.robot_x: float = 0.0
        self.robot_y: float = 0.0
        self.robot_yaw: float = 0.0
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self._xy_pub = self.create_publisher(String, "/goal_xy", 10)

        # Setup MQTT client
        self._mqtt_connected = False
        self._mqtt_client = mqtt.Client()
        self._mqtt_client.username_pw_set(username, password)
        self._mqtt_client.on_connect = self._on_connect
        self._mqtt_client.on_disconnect = self._on_disconnect

        try:
            self._mqtt_client.connect_async(broker, port)
            self._mqtt_client.loop_start()
            self.get_logger().info(f"Connecting to MQTT broker {broker}:{port}...")
        except Exception as e:
            self.get_logger().error(f"Failed to start MQTT connection: {e}")

        self.get_logger().info("MapClickMqtt ready — click the map to navigate")

    def _on_connect(self, client, userdata, flags, rc):
        """Callback for when MQTT connects."""
        if rc == 0:
            self._mqtt_connected = True
            self.get_logger().info("MQTT successfully connected!")
        else:
            self.get_logger().error(f"MQTT connection failed with code {rc}")

    def _on_disconnect(self, client, userdata, rc):
        """Callback for when MQTT disconnects."""
        self._mqtt_connected = False
        self.get_logger().warn(f"MQTT disconnected with return code {rc}")

    def _odom_cb(self, msg: Odometry) -> None:
        """Update robot position and yaw from /odom."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def send_goal(self, x: float, y: float) -> None:
        """Publish a JSON payload goal via MQTT and ROS 2 topic."""

        # ROS 2 Publisher
        xy_msg = String()
        xy_msg.data = json.dumps({"x": x, "y": y})
        self._xy_pub.publish(xy_msg)
        self.get_logger().info(f"ROS 2 Goal → /goal_xy : {xy_msg.data}")

        if self._mqtt_connected:
            payload = {
                "Position x": x,
                "Position y": y,
                "Time Stamped": int(time.time()),
            }
            try:
                self._mqtt_client.publish(self._topic, json.dumps(payload))
                self.get_logger().info(f"MQTT Goal → x={x:.3f} m  y={y:.3f} m")
            except Exception as e:
                self.get_logger().error(f"Failed to publish MQTT message: {e}")
        else:
            self.get_logger().warn("Cannot send goal via MQTT: MQTT not connected")

    def destroy_node(self):
        """Override to clean up MQTT client."""
        self._mqtt_client.loop_stop()
        self._mqtt_client.disconnect()
        super().destroy_node()


# ── Main ──────────────────────────────────────────────────────────────────────


def main(args=None):
    rclpy.init(args=args)
    node = MapClickMqttNode()

    map_bgr = cv2.cvtColor(_MAP_IMG, cv2.COLOR_GRAY2BGR)

    # ── Shared navigation state ───────────────────────────────────────────────
    goal_map_px = None
    last_cam_click = None  # (x, y) pixel on the camera frame

    # ── Open webcam ───────────────────────────────────────────────────────────
    cap = cv2.VideoCapture(0)
    cam_ok = cap.isOpened()
    if not cam_ok:
        node.get_logger().error(
            "Cannot open webcam (device 2) — continuing with map window only"
        )

    # ── Mouse callback: Map window ────────────────────────────────────────────
    def on_map_mouse(event, col, row, flags, param):
        nonlocal goal_map_px
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        x_m, y_m = map_px_to_m(col, row)
        goal_map_px = (col, row)
        node.get_logger().info(f"Click ({col},{row}) → map ({x_m:.3f}, {y_m:.3f}) m")
        node.send_goal(x_m, y_m)

    # ── Mouse callback: Camera window ─────────────────────────────────────────
    def on_cam_mouse(event, x, y, flags, param):
        nonlocal goal_map_px, last_cam_click
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if H_CAM is None:
            node.get_logger().error("Homography not computed")
            return
        last_cam_click = (x, y)
        px = np.array([[[float(x), float(y)]]], dtype=np.float32)
        world = cv2.perspectiveTransform(px, H_CAM)
        x_m, y_m = float(world[0][0][0]), float(world[0][0][1])
        node.get_logger().info(
            f"Camera click ({x},{y}) → world ({x_m:.3f}, {y_m:.3f}) m"
        )
        node.send_goal(x_m, y_m)
        # Update map marker
        goal_map_px = map_m_to_px(x_m, y_m)

    cv2.namedWindow("Map MQTT - mapuse28")
    cv2.setMouseCallback("Map MQTT - mapuse28", on_map_mouse)

    if cam_ok:
        cv2.namedWindow("Camera MQTT")
        cv2.setMouseCallback("Camera MQTT", on_cam_mouse)

    while rclpy.ok():
        # ── Draw map with overlays ────────────────────────────────────────────
        m = map_bgr.copy()

        # HUD text
        cv2.putText(
            m,
            "Click to navigate | Q = quit",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            1,
        )

        # MQTT Connection Status
        if node._mqtt_connected:
            cv2.putText(
                m,
                "MQTT: CONNECTED",
                (10, 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
            )
        else:
            cv2.putText(
                m,
                "MQTT: DISCONNECTED",
                (10, 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255),
                2,
            )

        # Navigation goal (red circle + cross)
        if goal_map_px:
            cv2.circle(m, goal_map_px, 9, (0, 0, 255), -1)
            cv2.drawMarker(
                m,
                goal_map_px,
                (0, 0, 255),
                cv2.MARKER_CROSS,
                22,
                2,
            )

        # Robot position (green circle + heading arrow)
        rx, ry = map_m_to_px(node.robot_x, node.robot_y)
        cv2.circle(m, (rx, ry), 8, (0, 210, 0), -1)
        ax = int(rx + 22 * math.cos(node.robot_yaw))
        ay = int(ry - 22 * math.sin(node.robot_yaw))
        cv2.arrowedLine(m, (rx, ry), (ax, ay), (0, 255, 0), 2, tipLength=0.35)

        # Legend (bottom-left)
        lx, ly = 10, MAP_H - 45
        cv2.circle(m, (lx + 6, ly), 5, (0, 210, 0), -1)
        cv2.putText(
            m,
            "Robot",
            (lx + 15, ly + 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 210, 0),
            1,
        )
        cv2.circle(m, (lx + 6, ly + 16), 5, (0, 0, 255), -1)
        cv2.putText(
            m,
            "Goal",
            (lx + 15, ly + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 0, 255),
            1,
        )

        cv2.imshow("Map MQTT - mapuse28", m)

        # ── Draw camera feed with overlays ────────────────────────────────────
        if cam_ok:
            ret, frame = cap.read()
            if ret:
                # HUD text
                cv2.putText(
                    frame,
                    "Click camera to navigate | Q = quit",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

                # MQTT Connection Status
                if node._mqtt_connected:
                    cv2.putText(
                        frame,
                        "MQTT: CONNECTED",
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )
                else:
                    cv2.putText(
                        frame,
                        "MQTT: DISCONNECTED",
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 0, 255),
                        2,
                    )

                # Last clicked position (red circle)
                if last_cam_click is not None:
                    cv2.circle(frame, last_cam_click, 10, (0, 0, 255), -1)
                    cv2.drawMarker(
                        frame,
                        last_cam_click,
                        (0, 0, 255),
                        cv2.MARKER_CROSS,
                        24,
                        2,
                    )

                cv2.imshow("Camera MQTT", frame)

        rclpy.spin_once(node, timeout_sec=0.01)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key == ord("c"):
            goal_map_px = None
            last_cam_click = None

    cv2.destroyAllWindows()
    if cam_ok:
        cap.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
