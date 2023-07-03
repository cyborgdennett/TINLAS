import os
import pathlib
import launch
import yaml
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    package_dir = get_package_share_directory("my_package")
    robot_description = pathlib.Path(os.path.join(package_dir, "resource", "supervisor.urdf")).read_text()

    # get yaml file path
    # with open(os.path.join(package_dir, "config", "config.yaml"), "r") as f:
    #     configuration = yaml.safe_load(f)

    webots = WebotsLauncher(world=os.path.join(package_dir, "worlds", "my_world.wbt"))

    supervisor_driver = Node(
        package="webots_ros2_driver",
        executable="driver",
        output="screen",
        namespace="supervisor2",
        additional_env={"WEBOTS_CONTROLLER_URL": controller_url_prefix() + "supervisor2"},
        parameters=[
            {"robot_description": robot_description},
        ],
    )
    s3 = Node(
        package="webots_ros2_driver",
        executable="driver",
        output="screen",
        namespace="supervisor3",
        additional_env={"WEBOTS_CONTROLLER_URL": controller_url_prefix() + "supervisor3"},
        parameters=[
            {"robot_description": robot_description},
        ],
    )

    return LaunchDescription(
        [
            webots,
            supervisor_driver,
            s3,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
