import rclpy 
from rclpy.node import Node
from rclpy.action import action

from nav2_msgs.action import NavigateThroughPoses

class Nav(Node):
    def __init__(self):
        super().__init__('Nav_Node')
        self.
