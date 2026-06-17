"""
sim_nav.launch.py — Gazebo-free RViz2 simulation for Nav2 testing

No Gazebo, no real robot required.  Runs:
  loopback_sim       — cmd_vel → odom + TF  (map→odom→base_footprint)
  map_server         — serves mapuse6.yaml
  Nav2 stack         — controller / planner / BT / behaviours
                       (collision_monitor excluded — not needed without a scanner)
  robot_state_pub    — TB3 URDF for RViz2 robot model
  rviz2              — Nav2 default view
  cam_nav            — click camera feed → /goal_pose

cmd_vel chain (collision_monitor removed):
  controller_server ─► cmd_vel_nav ─► velocity_smoother ─► cmd_vel ─► loopback_sim

Usage:
  ros2 launch mqtt_test sim_nav.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory("mqtt_test")
    nav2_dir = get_package_share_directory("nav2_bringup")
    tb3_dir = get_package_share_directory("nav2_minimal_tb3_sim")

    map_yaml = os.path.join(pkg_share, "config", "mapuse28.yaml")
    sim_params_raw = os.path.join(pkg_share, "config", "sim_params.yaml")
    rviz_cfg = os.path.join(nav2_dir, "rviz", "nav2_default_view.rviz")

    with open(os.path.join(tb3_dir, "urdf", "turtlebot3_waffle.urdf")) as f:
        robot_desc = f.read()

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
    )

    # Rewrite params (no namespace substitution needed, but RewrittenYaml
    # handles type coercion cleanly)
    sim_params = RewrittenYaml(
        source_file=sim_params_raw,
        root_key="",
        param_rewrites={},
        convert_types=True,
    )

    # Common tf remappings nav2 nodes expect
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # ── Map server ────────────────────────────────────────────────────────────
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[sim_params, {"yaml_filename": map_yaml, "use_sim_time": False}],
        remappings=tf_remaps,
    )

    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["map_server"],
            }
        ],
    )

    # ── Nav2 navigation nodes (collision_monitor excluded) ────────────────────
    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[sim_params],
        remappings=tf_remaps + [("cmd_vel", "cmd_vel_nav")],
    )

    smoother = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[sim_params],
        remappings=tf_remaps,
    )

    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[sim_params],
        remappings=tf_remaps,
    )

    behaviors = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[sim_params],
        remappings=tf_remaps + [("cmd_vel", "cmd_vel_nav")],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[sim_params],
        remappings=tf_remaps,
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[sim_params],
        remappings=tf_remaps,
    )

    # velocity_smoother: subscribes cmd_vel_nav, publishes directly to cmd_vel
    # (cmd_vel_smoothed remapped → cmd_vel since collision_monitor is absent)
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[sim_params],
        remappings=tf_remaps
        + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
    )

    nav_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "smoother_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                    "velocity_smoother",
                ],
            }
        ],
    )

    # ── Loopback simulator ────────────────────────────────────────────────────
    loopback_sim = Node(
        package="mqtt_test",
        executable="loopback_sim",
        name="loopback_sim",
        output="screen",
    )

    # ── Robot URDF for RViz2 model ────────────────────────────────────────────
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": False}],
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            # Infrastructure
            robot_state_pub,
            loopback_sim,
            # Map
            map_server,
            map_lifecycle,
            # Navigation
            controller,
            smoother,
            planner,
            behaviors,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            nav_lifecycle,
            # UI — RViz2 only; run camera node separately:
            #   ros2 run mqtt_test click_nav
            rviz2,
        ]
    )
