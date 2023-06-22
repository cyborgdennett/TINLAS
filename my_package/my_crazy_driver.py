import rclpy
from geometry_msgs.msg import Twist, PointStamped, Point
from sensor_msgs.msg import Range


from math import cos, sin

from .pid_controller import pid_velocity_fixed_height_controller

FLYING_ATTITUDE = 0.5  # 0.5 in webots


class MyCrazyDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        # TODO make functionality to set id and make that change the aruco marker to the id
        self.id = 0

        rclpy.init(args=None)
        self.__node = rclpy.create_node("my_crazy_driver")

        self.__node.get_logger().info("Starting MyCrazyDriver")
        # Distance Sensors: Exists as ROS msg
        # self.__right_distance_sensor = self.__robot.getDevice('range_right')
        # self.__left_distance_sensor = self.__robot.getDevice('range_left')
        # self.__back_distance_sensor = self.__robot.getDevice('range_back')
        # self.__front_distance_sensor = self.__robot.getDevice('range_front')

        # self.__right_distance_sensor.enable(self.__robot.getBasicTimeStep())
        # self.__left_distance_sensor.enable(self.__robot.getBasicTimeStep())
        # self.__back_distance_sensor.enable(self.__robot.getBasicTimeStep())
        # self.__front_distance_sensor.enable(self.__robot.getBasicTimeStep())

        self.timestep = int(self.__robot.getBasicTimeStep())

        # IMU : does not exist as ROS msg
        self.__imu = self.__robot.getDevice("inertial unit")
        self.__imu.enable(self.timestep)

        # Gyro : does not exist as ROS msg
        self.__gyro = self.__robot.getDevice("gyro")
        self.__gyro.enable(self.timestep)

        # gps : exists as ROS msg
        # self.__gps = self.__robot.getDevice('gps')
        # self.__gps.enable(self.__robot.getBasicTimeStep())

        # motors
        self.__m1_motor = self.__robot.getDevice("m1_motor")
        self.__m2_motor = self.__robot.getDevice("m2_motor")
        self.__m3_motor = self.__robot.getDevice("m3_motor")
        self.__m4_motor = self.__robot.getDevice("m4_motor")

        # read sensor data from ROS topics
        self.__range_right_data = Range()
        self.__range_back_data = Range()
        self.__range_left_data = Range()
        self.__range_front_data = Range()
        self.__node.create_subscription(
            Range, "range_right", self.__range_right_callback, 1
        )
        self.__node.create_subscription(
            Range, "range_left", self.__range_left_callback, 1
        )
        self.__node.create_subscription(
            Range, "range_front", self.__range_front_callback, 1
        )
        self.__node.create_subscription(
            Range, "range_back", self.__range_back_callback, 1
        )

        self.__gps_data = PointStamped()
        self.__node.create_subscription(PointStamped, "gps", self.__gps_callback, 1)

        self.__cam_pos_data = Point()
        self.__node.create_subscription(
            Point, f"aruco/feducial{self.id}", self.__cam_pos_callback, 1
        )

        # init motors
        self.__m1_motor.setPosition(float("inf"))
        self.__m1_motor.setVelocity(-1)
        self.__m2_motor.setPosition(float("inf"))
        self.__m2_motor.setVelocity(1)
        self.__m3_motor.setPosition(float("inf"))
        self.__m3_motor.setVelocity(-1)
        self.__m4_motor.setPosition(float("inf"))
        self.__m4_motor.setVelocity(1)

        ## Initialize pid variables

        self.past_x_global = 0
        self.past_y_global = 0
        self.past_time = self.__robot.getTime()

        # Initialize Crazyflie velocity PID controller

        self.PID_CF = pid_velocity_fixed_height_controller()
        # self.PID_update_last_time = self.__robot.getTime()
        # self.sensor_read_last_time = self.__robot.getTime()

        self.height_desired = FLYING_ATTITUDE

        # desired movement variables
        self.forward_desired = 0
        self.sideways_desired = 0
        self.yaw_desired = 0
        self.height_diff_desired = 0

        self.__node.create_subscription(
            Twist, f"target_pose{self.id}", self.__target_pose_callback, 1
        )

        self.__target_position_data = Point()
        self.__node.create_subscription(
            Point, f"target_position{self.id}", self.__target_position_callback, 1
        )

        # set upward veolcity to to 0.5 m/s
        self.height_diff_desired = 0.1
        # make timer to stop initial height change
        self.__node.create_timer(5, self.__init_height_callback)

        self.__node.get_logger().info("finished init")

    def __init_height_callback(self):
        self.height_diff_desired = 0

    # MSG type is Point

    def __cam_pos_callback(self, cam_pos):
        self.__cam_pos_data = cam_pos
        # self.__node.get_logger().info(f"cam_pos: {cam_pos}")
        # self.__node.get_logger().info(f"gps_pos: {self.__gps_data}")

    # MSG type is Twist

    def __target_pose_callback(self, target_pose):
        # TODO: make sure the maximum speed is not exceeded
        self.forward_desired = target_pose.linear.x
        self.sideways_desired = target_pose.linear.y
        self.height_diff_desired = target_pose.linear.z
        self.yaw_desired = target_pose.angular.y

    # MSG type is Range

    def __range_right_callback(self, range):
        self.__range_right_data = range

    def __range_left_callback(self, range):
        self.__range_left_data = range

    def __range_front_callback(self, range):
        self.__range_front_data = range

    def __range_back_callback(self, range):
        self.__range_back_data = range

    # MSG type is PointStamped

    def __gps_callback(self, gps_data):
        self.__gps_data = gps_data

    # MSG type is Point

    def __target_position_callback(self, target_position):
        self.__target_position_data = target_position

    # Runs every timestep in webots

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        dt = self.__robot.getTime() - self.past_time

        # read sensor data
        roll, pitch, yaw = self.__imu.getRollPitchYaw()
        # imuData = self.__imu.getRollPitchYaw()
        # roll = imuData[0]
        # pitch = imuData[1]
        # yaw = imuData[2]
        yaw_rate = self.__gyro.getValues()[2]
        # TODO: get the altitude with the IR sensors calculation mixed with imu
        altitude = self.__gps_data.point.z
        # TODO: remove gps and replace with the camera module xy
        # cam_pos is [0 .. 100, 0 .. 100] (in webots cam)
        # cam_pos is [0 .. 100, 0 .. 50] (in real world cam)
        # webots is [-0.5 .. 0.5, -0.5 .. 0.5]
        # translation function is: x = 2*campos.x - 100, y = campos.y - 50

        # sizediff = 100 / 1
        # halfsizediff = sizediff / 2
        # x_global = self.__cam_pos_data.x / sizediff - halfsizediff
        # y_global = self.__cam_pos_data.y / sizediff - halfsizediff
        x_global = self.__gps_data.point.x
        y_global = self.__gps_data.point.y
        v_x_global = (x_global - self.past_x_global) / dt
        v_y_global = (y_global - self.past_y_global) / dt

        ## Get body fixed velocities
        cosyaw = cos(yaw)
        sinyaw = sin(yaw)
        v_x = v_x_global * cosyaw + v_y_global * sinyaw
        v_y = -v_x_global * sinyaw + v_y_global * cosyaw

        self.height_desired += self.height_diff_desired * dt

        # update PID controller

        m1, m2, m3, m4 = self.PID_CF.pid(
            dt,
            self.forward_desired,
            self.sideways_desired,
            self.yaw_desired,
            self.height_desired,
            roll,
            pitch,
            yaw_rate,
            altitude,
            v_x,
            v_y,
        )

        # set motor velocities

        self.__m1_motor.setVelocity(m1)
        self.__m2_motor.setVelocity(m2)
        self.__m3_motor.setVelocity(m3)
        self.__m4_motor.setVelocity(m4)

        # update variables

        self.past_time = self.__robot.getTime()
        self.past_x_global = x_global
        self.past_y_global = y_global

        # self.__node.get_logger().info(f"m1: {m1},{m2},{m3},{m4}")
