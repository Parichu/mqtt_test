import rclpy
import paho.mqtt.client as mqtt
import json
import ssl
import time

from rclpy.node import Node
from geometry_msgs.msg import Pose

# MQTT Setup
broker = "xxx.xxx.x.x"  # Change IP Here
port = 8084
topic = "robot/nav_goal"
username = "parichu"
password = "1122"


# Class control Ros2 cmd_vel Topic
class ControlCmd(Node):
    def __init__(self):
        super().__init__("send_cmd_vel_with_mqtt")
        self.publishers_ = self.create_publisher(Pose, "fire/goal_pose", 10)
        self.timer = self.create_timer(1.5, self.timer_callback)
        self.pose_x = 0
        self.pose_y = 0
        self.time_stamp = 0
        # MQTT Client Setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(username, password)

        try:
            self.mqtt_client.connect(broker, port)
            self.mqtt_client.loop_start()  # Backgroud Start
            self.get_logger().info("Connect to MQTT Sucess")
        except Exception as e:
            self.get_logger().info(f"Can't Connect To MQTT Broker: {e}")

    def timer_callback(self):
        self.pose_x += 0.1
        self.pose_y += 0.1
        self.time_stamp

        # Set Angular Velocity (rad/s)

        # Craate Log_data Dictionary
        log_data = {
            "Position x": self.pose_x,  # Point in map
            "Position y": self.pose_y,  # Point in map
            "Time Stamped ": self.time_stamp,  # Time
        }

        # Send data to MQTT
        self.mqtt_client.publish(topic, json.dumps(log_data))

        # Display data
        self.get_logger().info(f"Publish Payload{log_data}")
        self.spd += 1

        def destroy_node(self):
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            super().destroy_node()


# Main func for start program
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
