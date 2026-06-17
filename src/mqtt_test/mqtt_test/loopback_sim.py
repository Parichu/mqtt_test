"""
loopback_sim — Gazebo-free robot simulator for Nav2 testing

Integrates /cmd_vel into a 2-D pose, then publishes:
  - nav_msgs/Odometry  →  /odom
  - TF:  map  → odom          (static identity at startup)
  - TF:  odom → base_link  (dynamic, updated 50 Hz)

No laser scanner is simulated — use sim_params.yaml which strips
scan layers from costmaps so Nav2 works with static-map only.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class LoopbackSim(Node):
    def __init__(self):
        super().__init__("loopback_sim")

        # Robot 2-D state in odom frame
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vyaw = 0.0
        self._last_cmd = self.get_clock().now()

        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._tf_br = TransformBroadcaster(self)
        self._static_br = StaticTransformBroadcaster(self)

        # Publish static map → odom (identity — robot starts at map origin)
        self._publish_map_odom_static()

        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.create_timer(0.02, self._step)  # 50 Hz

        self.get_logger().info(
            "LoopbackSim ready  —  robot starts at map (0.0, 0.0)  |  "
            "use RViz2 Nav2 goal to navigate"
        )

    # ── helpers ────────────────────────────────────────────────────────────────

    def _publish_map_odom_static(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.rotation.w = 1.0
        self._static_br.sendTransform(t)

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _cmd_cb(self, msg: Twist):
        self._vx = msg.linear.x
        self._vyaw = msg.angular.z
        self._last_cmd = self.get_clock().now()

    def _step(self):
        now = self.get_clock().now()
        dt = 0.02  # matches timer period

        # Stop if command is stale (>0.5 s — safety for real-robot parity)
        if (now - self._last_cmd).nanoseconds * 1e-9 > 0.5:
            self._vx = 0.0
            self._vyaw = 0.0

        # Integrate 2-D kinematics
        self._yaw += self._vyaw * dt
        self._x += self._vx * math.cos(self._yaw) * dt
        self._y += self._vx * math.sin(self._yaw) * dt

        qz = math.sin(self._yaw / 2.0)
        qw = math.cos(self._yaw / 2.0)
        stamp = now.to_msg()

        # TF: odom → base_link
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_br.sendTransform(tf)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.angular.z = self._vyaw
        self._odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = LoopbackSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
