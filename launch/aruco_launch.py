from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    side = perform_substitutions(context, [LaunchConfiguration('side')])

    aruco_marker_publisher_params = {
        'image_is_rectified': True,
        'marker_size': LaunchConfiguration('marker_size'),
        'reference_frame': LaunchConfiguration('reference_frame'),
        'camera_frame': side + '_hand_camera',
        'use_sim_time': True, # because webots uses sim time
    }

    aruco_marker_publisher = Node(
        package='aruco_ros',
        executable='marker_publisher',
        parameters=[aruco_marker_publisher_params],
        remappings=[('/camera_info', '/camera/top_view/camera_info'),
                    ('/image', '/camera/top_view')],
    )

    return [aruco_marker_publisher]


def generate_launch_description():

    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.05',
        description='Marker size in m. '
    )

    side_arg = DeclareLaunchArgument(
        'side', default_value='left',
        description='Side. ',
        choices=['left', 'right'],
    )

    reference_frame = DeclareLaunchArgument(
        'reference_frame', default_value='base',
        description='Reference frame. '
        'Leave it empty and the pose will be published wrt param parent_name. '
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(marker_size_arg)
    ld.add_action(side_arg)
    ld.add_action(reference_frame)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld


    # rqt = Node(
    #     package="rqt_gui",
    #     namespace="",
    #     executable="rqt_gui",
    #     name="rqt",
    #     arguments=[
    #         "--force-discover",
    #         "--perspective-file",
    #         [os.path.join(package_dir, "config", "swarm_control.perspective")],
    #     ],
    # )

    # tf_pub = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=[
    #         "--x", "0", "--y", "0", "--z", "1", "--yaw", "0", "--pitch", "0",
    #         "--roll", "0", "--frame-id", "map", "--child-frame-id", "/gps",
    #     ],
    # )