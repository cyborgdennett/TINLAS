import os
import pathlib
import launch
import yaml
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix

def generate_launch_description():
    package_dir = get_package_share_directory("my_package")
    
    params = [
        ("camera_topic", "/camera/top_view"),
        ("apriltag_array_topic", "/fiducial/apriltag_array"),
        ("apriltag_overlay_publish_enable", False),
        ("apriltag_overlay_topic", "/fducial/overlay"),                
    ]
    
    
    fiducial_follower = Node(
        package="my_package",
        executable="fiducial_follower",
        # parameters=params,
    )
    
    return LaunchDescription(
        [fiducial_follower]
        )