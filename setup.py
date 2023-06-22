from setuptools import find_packages, setup

package_name = "my_package"
data_files = []
data_files.append(
    ("share/ament_index/resource_index/packages", ["resource/" + package_name])
)
data_files.append(("share/" + package_name + "/launch", ["launch/robot_launch.py"]))
data_files.append(("share/" + package_name + "/launch", ["launch/crazy_launch.py"]))
data_files.append(("share/" + package_name + "/worlds", ["worlds/my_world.wbt"]))
data_files.append(("share/" + package_name + "/worlds", ["worlds/crazyflie_world.wbt"]))
data_files.append(
    (
        "share/" + package_name + "/protos/meshes",
        ["protos/meshes/cf2_assembly.dae", "protos/meshes/ccw_prop.dae"],
    )
)
data_files.append(
    ("share/" + package_name + "/protos/icons", ["protos/icons/Crazyflie.png"])
)
data_files.append(
    (
        "share/" + package_name + "/protos",
        [
            "protos/feducial.jpg",
            "protos/CrazyflieDistanceSensorDown.proto",
            "protos/CrazyflieDistanceSensorUp.proto",
            "protos/CrazyflieFeducial.proto",
            "protos/CrazyflieNoPhysics.proto",
        ],
    )
)
data_files.append(("share/" + package_name + "/resource", ["resource/my_robot.urdf"]))
data_files.append(("share/" + package_name + "/resource", ["resource/my_crazy.urdf"]))
data_files.append(("share/" + package_name + "/resource", ["resource/my_camera.urdf"]))
data_files.append(("share/" + package_name + "/config", ["config/video.rviz"]))
data_files.append(("share/" + package_name, ["package.xml"]))

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user.name@mail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "my_robot_driver = my_package.my_robot_driver:main",
            "my_crazy_driver = my_package.my_crazy_driver:main",
            "obstacle_avoider = my_package.obstacle_avoider:main",
            "feducial_follower = my_package.feducial_follower:main",
            "swarm_pathing = my_package.swarm_pathing:main",
            # 'pid_controller = my_package.pid_controller:pid_velocity_fixed_height_controller',
        ],
    },
)
