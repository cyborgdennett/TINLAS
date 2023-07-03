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

def generate_launch_description():    
    
    zmq_bridge = Node(
        package="my_package",
        executable="zmq_bridge",
    )
    
    return LaunchDescription(
        [zmq_bridge]
    )