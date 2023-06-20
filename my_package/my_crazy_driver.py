import rclpy
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import Range

from math import cos, sin

from .pid_controller import pid_velocity_fixed_height_controller
# from pid_controller import pid_velocity_fixed_height_controller

FLYING_ATTITUDE = 1.0

class MyCrazyDriver:
    def init(self, webots_node, properties):
        
        self.__robot = webots_node.robot
        
        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_crazy_driver')
        
        self.__node.get_logger().info('Starting MyCrazyDriver')
        # Distance Sensors: Exists as ROS msg
        # self.__right_distance_sensor = self.__robot.getDevice('range_right')
        # self.__left_distance_sensor = self.__robot.getDevice('range_left')
        # self.__back_distance_sensor = self.__robot.getDevice('range_back')
        # self.__front_distance_sensor = self.__robot.getDevice('range_front')
                
        # self.__right_distance_sensor.enable(self.__robot.getBasicTimeStep())    
        # self.__left_distance_sensor.enable(self.__robot.getBasicTimeStep()) 
        # self.__back_distance_sensor.enable(self.__robot.getBasicTimeStep())
        # self.__front_distance_sensor.enable(self.__robot.getBasicTimeStep())    
        counter = 0
        # IMU : does not exist as ROS msg
        self.__imu = self.__robot.getDevice('inertial unit')
        
        
        self.__imu.enable(int(self.__robot.getBasicTimeStep()))
        
        
        # Gyro : does not exist as ROS msg
        self.__gyro = self.__robot.getDevice('gyro')
        self.__gyro.enable(int(self.__robot.getBasicTimeStep()))
        
                
        # gps : exists as ROS msg
        # self.__gps = self.__robot.getDevice('gps')
        # self.__gps.enable(self.__robot.getBasicTimeStep())
        
        # motors
        self.__m1_motor = self.__robot.getDevice('m1_motor')
        self.__m2_motor = self.__robot.getDevice('m2_motor')
        self.__m3_motor = self.__robot.getDevice('m3_motor')
        self.__m4_motor = self.__robot.getDevice('m4_motor')
        
        
        # init motors  
        self.__m1_motor.setPosition(float('inf'))
        self.__m1_motor.setVelocity(0)
        self.__m2_motor.setPosition(float('inf'))
        self.__m2_motor.setVelocity(0)
        self.__m3_motor.setPosition(float('inf'))
        self.__m3_motor.setVelocity(0)
        self.__m4_motor.setPosition(float('inf'))
        self.__m4_motor.setVelocity(0)

        # self.__target_twist = Twist()
        
        ## Initialize variables

        self.past_x_global = 0
        self.past_y_global = 0
        self.past_time = self.__robot.getTime()
        
        # Crazyflie velocity PID controller
        self.PID_CF = pid_velocity_fixed_height_controller()
        self.PID_update_last_time = self.__robot.getTime()
        self.sensor_read_last_time = self.__robot.getTime()
        
        self.height_desired = FLYING_ATTITUDE
        
        # get movement commands
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        # read sensor data
        self.__range_right_data = Range()
        self.__range_back_data = Range()
        self.__range_left_data = Range()
        self.__range_front_data = Range()
        self.__node.create_subscription(Range, 'range_right', self.__range_right_callback, 1)
        self.__node.create_subscription(Range, 'range_left', self.__range_left_callback, 1)
        self.__node.create_subscription(Range, 'range_front', self.__range_front_callback, 1)
        self.__node.create_subscription(Range, 'range_back', self.__range_back_callback, 1)
        
        self.__gps_data = PointStamped()
        self.__node.create_subscription(PointStamped, 'gps', self.__gps_callback, 1)
        
        self.__node.get_logger().info("finished init")  
        
        
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
        
    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist
        
    def __target_position_callback(self, target_position):
        return
    
    
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
    
        dt = self.__robot.getTime() - self.past_time
        # actual_state = {}
        
        # read sensor data
        imuData = self.__imu.getRollPitchYaw()
        roll = imuData[0]
        pitch = imuData[1]
        yaw = imuData[2]
        yaw_rate = self.__gyro.getValues()[2]
        # TODO: remove gps and replace with the camera module xy
        altitude = self.__gps_data.point.z
        x_global = self.__gps_data.point.x
        v_x_global = (x_global - self.past_x_global)/dt
        y_global = self.__gps_data.point.y
        v_y_global = (y_global - self.past_y_global)/dt
        
        ## Get body fixed velocities
        cosyaw = cos(yaw)
        sinyaw = sin(yaw)
        v_x = v_x_global * cosyaw + v_y_global * sinyaw
        v_y = - v_x_global * sinyaw + v_y_global * cosyaw

        ## Initialize values
        # desired_state = [0, 0, 0, 0]
        forward_desired = 0.5 # test forward
        # forward_desired = 0
        sideways_desired = 0
        yaw_desired = 0
        height_diff_desired = 0
        
        self.height_desired += height_diff_desired * dt
        
        # update PID controller
        
        [m1,m2,m3,m4] = self.PID_CF.pid(dt, forward_desired, sideways_desired,
                                yaw_desired, self.height_desired,
                                roll, pitch, yaw_rate,
                                altitude, v_x, v_y)
        
        # set motor velocities
        
        self.__m1_motor.setVelocity(m1)
        self.__m2_motor.setVelocity(m2)
        self.__m3_motor.setVelocity(m3)
        self.__m4_motor.setVelocity(m4)
        
        # update variables

        self.past_time = self.__robot.getTime()
        self.past_x_global = x_global
        self.past_y_global = y_global

