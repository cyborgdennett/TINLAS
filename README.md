# TINLAS
Ros2 Humble
# Requirements
* Webots 2023A
* Ubuntu 22.04 LTS
* webots-ros2
* ROS2 Humble-desktop
# Install
source ROS
``` bash
source /opt/ros/humble/setup.bash
```
get files in workspace
``` bash
mkdir -p ros_ws/src 
cd ros_ws/src
git clone /* THIS REPO */
```
[setup webots ros](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html)
init rosdep (in ws base)
``` bash
cd ..
sudo rosdep init
```
get dependencies
``` bash
rosdep install --from-paths src -y --ignore-src
```
get opencv
''' bash
pip install opencv-contrib-python
'''

Install Crazyflie-firmware
``` bash
sudo apt-get install make gcc-arm-none-eabi
sudo apt-get install swig

cd
git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git
cd crazyflie-firmware
make cf2_defconfig
make -j 12
make cf2_defconfig
make bindings_python
export PYTHONPATH=/home/casper/crazyflie-firmware/build:$PYTHONPATH
```

# Tutorial
source ROS
``` bash
source /opt/ros/humble/setup.bash
```
build
``` bash
colcon build
```
source and run
``` bash
source install/local_setup.bash
ros2 launch my_package crazy_launch.py
```

While developing, and you change code and want to start up again use:
``` bash
colcon build;source install/local_setup.bash;ros2 launch my_package crazy_launch.py
```
in the ros_ws
