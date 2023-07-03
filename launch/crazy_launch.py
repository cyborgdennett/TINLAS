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

    robot_description = pathlib.Path(
        os.path.join(package_dir, "resource", "my_crazy.urdf")
    ).read_text()
    camera_description = pathlib.Path(
        os.path.join(package_dir, "resource", "my_camera.urdf")
    ).read_text()

    # get yaml file path
    configuration = {}
    with open(os.path.join(package_dir, "config", "config.yaml"), "r") as f:
        configuration = yaml.safe_load(f)

    world = LaunchConfiguration("world")

    webots = WebotsLauncher(
        # world=os.path.join(package_dir, 'worlds', 'crazyflie_world.wbt')
        world=PathJoinSubstitution([package_dir, "worlds", world])
    )

    feducial_follower = Node(
        package="my_package",
        executable="feducial_follower",
        # parameters=configuration["feducial_follower"],
    )

    my_crazy_driver = Node(
        package="webots_ros2_driver",
        executable="driver",
        output="screen",
        additional_env={"WEBOTS_CONTROLLER_URL": controller_url_prefix() + "my_crazy"},
        parameters=[
            {"robot_description": robot_description},
        ],
    )
    my_camera_driver = Node(
        package="webots_ros2_driver",
        executable="driver",
        additional_env={"WEBOTS_CONTROLLER_URL": controller_url_prefix() + "my_camera"},
        parameters=[
            {"robot_description": camera_description},
        ],
    )
    obstacle_avoider = Node(
        package="my_package",
        executable="obstacle_avoider",
    )
    aruco_ros = Node(package="aruco_ros", executable="", parameters=[])

    rviz = Node(
        package="rviz2",
        namespace="",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", [os.path.join(package_dir, "config", "video.rviz")]],
    )

    rqt = Node(
        package="rqt_gui",
        namespace="",
        executable="rqt_gui",
        name="rqt",
        arguments=[
            "--force-discover",
            "--perspective-file",
            [os.path.join(package_dir, "config", "swarm_control.perspective")],
        ],
    )

    tf_pub = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "1",
            "--yaw",
            "0",
            "--pitch",
            "0",
            "--roll",
            "0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "/gps",
        ],
    )

    swarm_pathing = Node(
        package="my_package",
        executable="swarm_pathing",
        # ros_arguments=
        # arguments=['-d', [os.path.join(package_dir, 'config', 'video.rviz')]]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="crazyflie_world.wbt",
                description="Choose one of the world files from `/my_package/worlds` directory",
            ),
            webots,
            my_crazy_driver,
            my_camera_driver,
            swarm_pathing,
            # obstacle_avoider,
            # feducial_follower,
            aruco_ros,
            tf_pub,
            rviz,
            rqt,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=rviz,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
