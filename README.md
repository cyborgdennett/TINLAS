# TINLAS
Ros2 Humble

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
