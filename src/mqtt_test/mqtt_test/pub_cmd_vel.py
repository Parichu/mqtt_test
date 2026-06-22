"""
pub_cmd_vel Node — Publishes robot pose telemetry to MQTT broker and ROS /fire/goal_pose topic

Published Topics:
  /goal_xy  (std_msgs/String)  — JSON goal: {"x": float, "y": float}

MQTT Published:
  <mqtt_topic>  JSON: {"Position x": float, "Position y": float, "Time Stamped": int}

Parameters:
  mqtt_broker    (str,   default: '172.20.10.6')   — MQTT broker hostname or IP
  mqtt_port      (int,   default: 1883)             — MQTT broker TCP port
  mqtt_topic     (str,   default: 'robot/nav_goal') — MQTT topic to publish telemetry
  mqtt_username  (str,   default: 'parichu')        — MQTT username
  mqtt_password  (str,   default: '1122')           — MQTT password
  timer_period   (float, default: 1.5)              — Timer interval in seconds
  goal_x         (float, default: 9.7)              — Target X position (metres)
  goal_y         (float, default: 5.4)              — Target Y position (metres)
"""

import json

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class ControlCmd(Node):
    def __init__(self):
        super().__init__("send_cmd_vel_with_mqtt")

        self.declare_parameter("mqtt_broker", "172.20.10.6")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("mqtt_topic", "robot/nav_goal")
        self.declare_parameter("mqtt_username", "parichu")
        self.declare_parameter("mqtt_password", "1122")
        self.declare_parameter("timer_period", 1.5)
        self.declare_parameter("goal_x", 9.7)
        self.declare_parameter("goal_y", 5.4)

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
        timer_period = (
            self.get_parameter("timer_period").get_parameter_value().double_value
        )
        self._goal_x = self.get_parameter("goal_x").get_parameter_value().double_value
        self._goal_y = self.get_parameter("goal_y").get_parameter_value().double_value

        self._publisher = self.create_publisher(String, "/goal_xy", 10)
        self._timer = self.create_timer(timer_period, self._timer_callback)

        self._time_stamp = 0
        self._mqtt_connected = False

        self._mqtt_client = mqtt.Client()
        self._mqtt_client.username_pw_set(username, password)
        self._mqtt_client.on_disconnect = self._on_disconnect

        try:
            self._mqtt_client.connect(broker, port)
            self._mqtt_client.loop_start()
            self._mqtt_connected = True
            self.get_logger().info(f"Connected to MQTT broker at {broker}:{port}")
        except ConnectionRefusedError:
            self.get_logger().error(
                f"Connection refused — broker not reachable at {broker}:{port}. "
                "Check broker is running and credentials are correct."
            )
        except Exception as e:
            self.get_logger().error(f"MQTT connect failed: {e}")

    def _timer_callback(self):
        self._time_stamp += 1

        # Publish ROS goal (JSON string matching nav.py's expected format)
        goal_msg = String()
        goal_msg.data = json.dumps({"x": self._goal_x, "y": self._goal_y})
        self._publisher.publish(goal_msg)

        # Publish MQTT telemetry
        payload = {
            "Position x": self._goal_x,
            "Position y": self._goal_y,
            "Time Stamped": self._time_stamp,
        }
        if self._mqtt_connected:
            self._mqtt_client.publish(self._topic, json.dumps(payload))
        self.get_logger().info(f"Published: {payload}")

    def _on_disconnect(self, client, userdata, rc):
        self._mqtt_connected = False
        if rc != 0:
            self.get_logger().warn(
                f"Unexpected MQTT disconnect (rc={rc}), attempting reconnect..."
            )

    def destroy_node(self):
        self._mqtt_client.loop_stop()
        self._mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControlCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
