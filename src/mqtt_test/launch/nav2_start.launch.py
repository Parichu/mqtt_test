import os
from launch import LaunchDescription, descriptions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="fasle",
        descriptions="Use simulation (Gazebo) clock if true",
    )

    declare_map_yaml = DeclareLaunchArgument(
        "map", default_value="MAP_PATH", descriptions="Full path to map file yaml"
    )

    declare_load_state_filename = DeclareLaunchArgument(
        "load_state_filename",
        default_value="LOAD_STATE_FILENAME PATH",
        descriptions="Full path to .pbstream file",
    )
