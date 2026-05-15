import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# --- 1. YOUR CALIBRATION DATA ---
# Replace with the output from Script 1
image_points = np.array(
    [
        [0, 1],  # Top-Left
        [1277, 0],  # Top-Right
        [0, 718],  # Bottom-Left
        [1277, 718],  # Bottom-Right
    ],
    dtype=np.float32,
)

# Set the real Gazebo grid meters that match the pixels above
map_points = np.array(
    [
        [2.0, 2.0],  # Top-Left (2m forward, 2m left)
        [2.0, -2.0],  # Top-Right (2m forward, 2m right)
        [-2.0, 2.0],  # Bottom-Left
        [-2.0, -2.0],  # Bottom-Right
    ],
    dtype=np.float32,
)

H, _ = cv2.findHomography(image_points, map_points)


# --- 2. ROS 2 NODE SETUP ---
class VisionCommander(Node):
    def __init__(self):
        super().__init__("vision_commander")
        # TurtleBot3 Nav2 listens to /goal_pose
        self.publisher_ = self.create_publisher(PoseStamped, "/goal_pose", 10)

    def send_goal(self, x_meter, y_meter):
        msg = PoseStamped()
        msg.header.frame_id = "map"  # Tell Nav2 this is a global map coordinate
        msg.header.stamp = self.get_clock().now().to_msg()

        # Set Position
        msg.pose.position.x = float(x_meter)
        msg.pose.position.y = float(y_meter)
        msg.pose.position.z = 0.0

        # Set Orientation (Flat, facing forward)
        msg.pose.orientation.w = 1.0

        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent Goal: X={x_meter:.2f}m, Y={y_meter:.2f}m")


# --- 3. OPENCV & MAIN LOOP ---
def main(args=None):
    rclpy.init(args=args)
    commander = VisionCommander()

    cap = cv2.VideoCapture(0)

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Math: Pixel to Meters
            px = np.array([[[x, y]]], dtype=np.float32)
            m_coords = cv2.perspectiveTransform(px, H)
            target_x, target_y = m_coords[0][0]

            # Action: Send to ROS 2
            commander.send_goal(target_x, target_y)

    cv2.namedWindow("TB3 Command Center")
    cv2.setMouseCallback("TB3 Command Center", on_click)

    print("Click on the camera feed to move the TurtleBot3!")

    while rclpy.ok():
        ret, frame = cap.read()
        if not ret:
            break

        cv2.imshow("TB3 Command Center", frame)

        # Spin ROS 2 to handle messages, but timeout fast so video doesn't freeze
        rclpy.spin_once(commander, timeout_sec=0.01)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    commander.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
