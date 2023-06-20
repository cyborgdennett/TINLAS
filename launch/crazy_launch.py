import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    package_dir = get_package_share_directory('my_package')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'my_crazy.urdf')).read_text()
    camera_description = pathlib.Path(os.path.join(package_dir, 'resource', 'my_camera.urdf')).read_text()

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'crazyflie_world.wbt')
    )

    my_crazy_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env=
            {'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'my_crazy'},
        parameters=[
            {'robot_description': robot_description},
        ]
    )
    my_camera_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'my_camera'},
        parameters=[
            {'robot_description': camera_description},
        ]
    )
    obstacle_avoider = Node(
        package='my_package',
        executable='obstacle_avoider',
    )
    
    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(package_dir, 'config', 'video.rviz')]]
    )

    return LaunchDescription([
        webots,
        my_crazy_driver,
        my_camera_driver,
        # obstacle_avoider,
        rviz,
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
        )
    ])

