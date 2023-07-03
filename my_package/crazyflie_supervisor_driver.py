""" 
    This is the crazyflie supervisor driver module.
    It is based on the driver of [crazyflie_ros2_experimental](https://github.com/knmcguire/crazyflie_ros2_experimental), 
      and is used to control multiple drones.
    
    This module is used to control the crazyflie supervisor.
    It is used to spawn in and control the crazyflies, with the following functions:
        - spawn_crazyflie()
        - takeoff_crazyflie()
        - land_crazyflie()
        - set_crazyflie_position()
      
    The supervisor has 2 modes of operation:
        - PID mode (default)
            | In this mode, the supervisor will use the PID controller to control the crazyflies.
        - MPC mode
            | In this mode, the supervisor will use the MPC controller to control the crazyflies.
            | This is an experimental feature, and is not fully supported yet.

    The ROS publishers and subscribers are also defined in this module.
    The published topics are:
        - /crazyflie_supervisor/crazyflie_position
        - /crazyflie_supervisor/crazyflie_velocity
    The subscribed topics are:
        - /crazyflie_supervisor/crazyflie_command
    The actions are:
        - /crazyflie_supervisor/crazyflie_takeoff
        - /crazyflie_supervisor/crazyflie_land
        - /crazyflie_supervisor/crazyflie_spawn
        - /crazyflie_supervisor/crazyflie_destroy
"""
from typing import List
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from math import cos, sin, degrees, radians, pi
import sys
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# Change this path to your crazyflie-firmware folder
sys.path.append("/home/casper/crazyflie-firmware/build")
import cffirmware


class CrazyflieWebotsSupervisorDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        timestep = int(self.robot.getBasicTimeStep())


class CrazyflieWebotsDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        timestep = int(self.robot.getBasicTimeStep())

        self.supervisor = self.getSupervisor()

        ## Initialize motors
        self.m1_motor = self.robot.getDevice("m1_motor")
        self.m1_motor.setPosition(float("inf"))
        self.m1_motor.setVelocity(-1)
        self.m2_motor = self.robot.getDevice("m2_motor")
        self.m2_motor.setPosition(float("inf"))
        self.m2_motor.setVelocity(1)
        self.m3_motor = self.robot.getDevice("m3_motor")
        self.m3_motor.setPosition(float("inf"))
        self.m3_motor.setVelocity(-1)
        self.m4_motor = self.robot.getDevice("m4_motor")
        self.m4_motor.setPosition(float("inf"))
        self.m4_motor.setVelocity(1)

        self.target_twist = Twist()

        ## Initialize Sensors
        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(timestep)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(timestep)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(timestep)
        self.range_front = self.robot.getDevice("range_front")
        self.range_front.enable(timestep)
        self.range_left = self.robot.getDevice("range_left")
        self.range_left.enable(timestep)
        self.range_back = self.robot.getDevice("range_back")
        self.range_back.enable(timestep)
        self.range_right = self.robot.getDevice("range_right")
        self.range_right.enable(timestep)

        # self.lidar = self.robot.getDevice("lidar")
        # self.lidar.enable(timestep)

        ## Intialize Variables
        self.past_x_global = 0
        self.past_y_global = 0
        self.past_z_global = 0
        self.past_time = self.robot.getTime()

        self.first_pos = True
        self.first_x_global = 0.0
        self.first_y_global = 0.0

        rclpy.init(args=None)
        self.node = rclpy.create_node("crazyflie_webots_driver")
        self.node.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 1)
        self.laser_publisher = self.node.create_publisher(LaserScan, "scan", 10)
        self.odom_publisher = self.node.create_publisher(Odometry, "odom", 10)

        self.tfbr = TransformBroadcaster(self.node)

        self.msg_laser = LaserScan()
        self.node.create_timer(1.0 / 30.0, self.publish_laserscan_data)

        cffirmware.controllerPidInit()
        self.node.get_logger().info("Crazyflie Webots Driver Initialized")

    def publish_laserscan_data(self):
        front_range = float(self.range_front.getValue() / 1000.0)

        back_range = float(self.range_back.getValue() / 1000.0)
        left_range = float(self.range_left.getValue() / 1000.0)
        right_range = float(self.range_right.getValue() / 1000.0)
        max_range = 3.49
        if front_range > max_range:
            front_range = 0.0
        if left_range > max_range:
            left_range = 0.0
        if right_range > max_range:
            right_range = 0.0
        if back_range > max_range:
            back_range = 0.0

        self.msg_laser = LaserScan()
        self.msg_laser.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        self.msg_laser.header.frame_id = "base_link"
        self.msg_laser.range_min = 0.1
        self.msg_laser.range_max = max_range
        # print('print',back_range, left_range, front_range, right_range, back_range)
        self.msg_laser.ranges = [
            back_range,
            left_range,
            front_range,
            right_range,
            back_range,
        ]
        # self.msg_laser.ranges = [max_range, max_range, max_range, max_range, max_range]

        self.msg_laser.angle_min = 0.5 * 2 * pi
        self.msg_laser.angle_max = -0.5 * 2 * pi
        self.msg_laser.angle_increment = -1.0 * pi / 2
        self.laser_publisher.publish(self.msg_laser)

    def cmd_vel_callback(self, twist):
        self.node.get_logger().info('cmd_vel_callback: "%s"' % twist)
        self.target_twist = twist

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        dt = self.robot.getTime() - self.past_time

        if self.first_pos is True:
            self.first_x_global = self.gps.getValues()[0]
            self.first_y_global = self.gps.getValues()[1]
            self.first_pos = False

        ## Get measurements
        roll = self.imu.getRollPitchYaw()[0]
        pitch = self.imu.getRollPitchYaw()[1]
        yaw = self.imu.getRollPitchYaw()[2]
        roll_rate = self.gyro.getValues()[0]
        pitch_rate = self.gyro.getValues()[1]
        yaw_rate = self.gyro.getValues()[2]
        x_global = self.gps.getValues()[0] - self.first_x_global
        vx_global = (x_global - self.past_x_global) / dt
        y_global = self.gps.getValues()[1] - self.first_y_global
        vy_global = (y_global - self.past_y_global) / dt
        z_global = self.gps.getValues()[2]
        vz_global = (z_global - self.past_z_global) / dt

        """
        ranges = self.lidar.getLayerRangeImage(0)
        if ranges:
            msg = LaserScan()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link_rotated'
            msg.angle_min = -0.5 * pi
            msg.angle_max = 0.5 * pi
            msg.angle_increment = pi / (self.lidar.getHorizontalResolution() - 1)
            #msg.scan_time = self.lidar.getSamplingPeriod() / 1000.0
            msg.range_min = self.lidar.getMinRange() 
            msg.range_max = self.lidar.getMaxRange()
            msg.ranges = ranges
           # self.laser_publisher.publish(msg)
            ##print(self.lidar.getFov())
            #print(ranges)"""

        q_base = tf_transformations.quaternion_from_euler(0, 0, yaw)
        odom = Odometry()
        odom.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x_global
        odom.pose.pose.position.y = y_global
        odom.pose.pose.position.z = 0.0

        # odom.pose.pose.orientation.x = q_base[0]
        # odom.pose.pose.orientation.y = q_base[1]
        # odom.pose.pose.orientation.z = q_base[2]
        # odom.pose.pose.orientation.w = q_base[3]
        odom.pose.pose.orientation.z = sin(yaw / 2)
        odom.pose.pose.orientation.w = cos(yaw / 2)

        self.odom_publisher.publish(odom)

        t_base = TransformStamped()
        t_base.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        t_base.header.frame_id = "odom"
        t_base.child_frame_id = "base_link"
        t_base.transform.translation.x = x_global
        t_base.transform.translation.y = y_global
        t_base.transform.translation.z = 0.0
        # t_base.transform.rotation.x = q_base[0]
        # t_base.transform.rotation.y = q_base[1]
        # t_base.transform.rotation.z = q_base[2]
        # t_base.transform.rotation.w = q_base[3]
        t_base.transform.rotation.z = sin(yaw / 2)
        t_base.transform.rotation.w = cos(yaw / 2)
        self.tfbr.sendTransform(t_base)

        ## Put measurement in state estimate
        # TODO replace these with a EKF python binding
        state = cffirmware.state_t()
        state.attitude.roll = degrees(roll)
        state.attitude.pitch = -degrees(pitch)
        state.attitude.yaw = degrees(yaw)
        state.position.x = x_global
        state.position.y = y_global
        state.position.z = z_global
        state.velocity.x = vx_global
        state.velocity.y = vy_global
        state.velocity.z = vz_global

        # Put gyro in sensor data
        sensors = cffirmware.sensorData_t()
        sensors.gyro.x = degrees(roll_rate)
        sensors.gyro.y = degrees(pitch_rate)
        sensors.gyro.z = degrees(yaw_rate)
        yawDesired = 0

        ## Fill in Setpoints
        setpoint = cffirmware.setpoint_t()
        setpoint.mode.z = cffirmware.modeAbs
        setpoint.position.z = 1.0
        setpoint.mode.yaw = cffirmware.modeVelocity
        # TODO: find out why this multipication is necessary...
        setpoint.attitudeRate.yaw = degrees(self.target_twist.angular.z) * 5
        setpoint.mode.x = cffirmware.modeVelocity
        setpoint.mode.y = cffirmware.modeVelocity
        setpoint.velocity.x = self.target_twist.linear.x
        setpoint.velocity.y = self.target_twist.linear.y
        setpoint.velocity_body = True

        ## Firmware PID bindings
        control = cffirmware.control_t()
        tick = 100  # this value makes sure that the position controller and attitude controller are always always initiated
        cffirmware.controllerPid(control, setpoint, sensors, state, tick)

        ##
        cmd_roll = radians(control.roll)
        cmd_pitch = radians(control.pitch)
        cmd_yaw = -radians(control.yaw)
        cmd_thrust = control.thrust

        ## Motor mixing
        motorPower_m1 = cmd_thrust - cmd_roll + cmd_pitch + cmd_yaw
        motorPower_m2 = cmd_thrust - cmd_roll - cmd_pitch - cmd_yaw
        motorPower_m3 = cmd_thrust + cmd_roll - cmd_pitch + cmd_yaw
        motorPower_m4 = cmd_thrust + cmd_roll + cmd_pitch - cmd_yaw

        scaling = 1000  ##Todo, remove necessity of this scaling (SI units in firmware)
        self.m1_motor.setVelocity(-motorPower_m1 / scaling)
        self.m2_motor.setVelocity(motorPower_m2 / scaling)
        self.m3_motor.setVelocity(-motorPower_m3 / scaling)
        self.m4_motor.setVelocity(motorPower_m4 / scaling)

        self.past_time = self.robot.getTime()
        self.past_x_global = x_global
        self.past_y_global = y_global
        self.past_z_global = z_global