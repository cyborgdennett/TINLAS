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
    fiducial_follower = Node(
        package="my_package",
        executable="fiducial_follower",
        # parameters=params,
    )
    swarm_pathing = Node(
        package="my_package",
        executable="swarm_pathing",
    )
    tf_pub = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ['--x', '0', '--y', '0', '--z', '1', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'map', '--child-frame-id', '/agent_1/gps']
    )
    return LaunchDescription(
        # [zmq_bridge, fiducial_follower, swarm_pathing]
        [zmq_bridge, fiducial_follower]
        # [swarm_pathing]
    )