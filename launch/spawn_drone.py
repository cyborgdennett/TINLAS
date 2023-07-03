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
from launch.substitution import Substitution
from launch.substitutions import TextSubstitution

package_dir = get_package_share_directory("my_package")
robot_description = robot_description = pathlib.Path(os.path.join(package_dir, "resource", "crazyflie.urdf")).read_text()


def gen_crazyflie_node(id, x, y):
    translation = f'{x} {y} 0.015'
    
    
    name = "agent_" + str(id)
    return Node(
        package="webots_ros2_driver",
        executable="driver",
        output="screen",
        namespace=name,
        additional_env={"WEBOTS_CONTROLLER_URL": controller_url_prefix() + name},
        parameters=[
            {"robot_description": robot_description},
        ])
    
def generate_launch_description():


    return LaunchDescription(
        [
            gen_crazyflie_node(i)
        ] 
    )
