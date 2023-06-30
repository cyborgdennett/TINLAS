from setuptools import find_packages, setup
import os


package_name = "my_package"

data_files = []
data_files.append(
    ("share/ament_index/resource_index/packages", ["resource/" + package_name])
)
data_files.append(("share/" + package_name, ["package.xml"]))

def package_files(data_files, directory_list, same_destination=False):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, directory if same_destination else "", path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files


data_files = package_files(data_files, ["launch"])
data_files = package_files(data_files, [
    "config",
    "resource",
    "protos",
    "protos/aruco",
    "protos/meshes",
    "worlds",
    "worlds/aruco",
    ], True)

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files, 
    # data_files=package_files(data_files, ["launch"]),
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
            "fiducial_follower = my_package.fiducial_follower:main",
            "swarm_pathing = my_package.swarm_pathing:main",
            "supervisor_driver = my_package.supervisor_driver:main",
            "crazyflie_driver = my_package.crazyflie_driver:main",
            "zmq_bridge = my_package.zmq_bridge:main",
            # 'pid_controller = my_package.pid_controller:pid_velocity_fixed_height_controller',
        ],
    },
)
