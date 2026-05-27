"""
Nav Node — Navigate robot to XY goal received via /goal_xy topic (MQTT JSON bridge)

Subscribed Topics:
  /goal_xy  (std_msgs/String)  JSON payload: {"x": float, "y": float}

Action Clients:
  navigate_to_pose  (nav2_msgs/NavigateToPose)

Parameters:
  goal_topic  (str,   default: '/goal_xy')  — ROS topic that carries MQTT goal payloads
  frame_id    (str,   default: 'map')       — Coordinate frame used for the goal pose
"""

import json

import rclpy
from rclpy.action.client import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


class NavNode(Node):
    def __init__(self):
        super().__init__("nav_node")

        self.declare_parameter("goal_topic", "/goal_xy")
        self.declare_parameter("frame_id", "map")

        goal_topic = self.get_parameter("goal_topic").get_parameter_value().string_value
        self._frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self._goal_sub = self.create_subscription(
            String, goal_topic, self._goal_callback, qos
        )

        self._is_navigating = False
        self.get_logger().info(f"NavNode ready — listening on [{goal_topic}]")

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _goal_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            x = float(data["x"])
            y = float(data["y"])
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().error(f"Invalid goal payload: {e}")
            return

        if self._is_navigating:
            self.get_logger().warn("Already navigating — new goal ignored.")
            return

        self._send_goal(x, y)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2.")
            self._is_navigating = False
            return

        self.get_logger().info("Goal accepted — robot is navigating...")
        goal_handle.get_result_async().add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f"Distance remaining: {dist:.2f} m", throttle_duration_sec=2.0
        )

    def _result_callback(self, future):
        status = future.result().status
        if status == 4:  # GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info("Goal reached successfully.")
        else:
            self.get_logger().warn(f"Navigation ended with status code: {status}")
        self._is_navigating = False

    # ── helpers ────────────────────────────────────────────────────────────────

    def _send_goal(self, x: float, y: float):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available.")
            return

        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0  # identity quaternion — no forced heading

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f"Sending goal → x={x}, y={y}")
        self._is_navigating = True

        future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)


def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
