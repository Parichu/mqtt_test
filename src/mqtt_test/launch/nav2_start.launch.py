"""
nav2_start.launch.py

Launches full Nav2 stack (map_server + AMCL + planner + controller + BT)
with mapuse6.yaml, RViz2, and the camera-to-goal node (cam_nav).

Usage:
  ros2 launch mqtt_test nav2_start.launch.py
  ros2 launch mqtt_test nav2_start.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mqtt_test")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    map_yaml = os.path.join(pkg_share, "config", "mapuse6.yaml")
    nav2_params = os.path.join(nav2_bringup_dir, "params", "nav2_params.yaml")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock (true for Gazebo)",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── Nav2 full stack (map_server + AMCL + planner + controller) ────────────
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
            "autostart": "true",
        }.items(),
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "rviz_launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ── Camera navigation node ─────────────────────────────────────────────────
    cam_nav_node = Node(
        package="mqtt_test",
        executable="cam_nav",
        name="cam_nav",
        output="screen",
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            nav2_bringup,
            rviz2,
            cam_nav_node,
        ]
    )
