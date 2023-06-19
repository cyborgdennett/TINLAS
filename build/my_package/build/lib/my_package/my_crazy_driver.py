import rclpy
from geometry_msgs.msg import Twist

class MyCrazyDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        
        # sensors
        self.__right_distance_sensor = self.__robot.getDevice('range_right')
        self.__left_distance_sensor = self.__robot.getDevice('range_left')
        self.__back_distance_sensor = self.__robot.getDevice('range_back')
        self.__front_distance_sensor = self.__robot.getDevice('range_front')
                
        self.__imu = self.__robot.getDevice('inertial_unit')
                
        # motors
        self.__m1_motor = self.__robot.getDevice('m1_motor')
        self.__m2_motor = self.__robot.getDevice('m2_motor')
        self.__m3_motor = self.__robot.getDevice('m3_motor')
        self.__m4_motor = self.__robot.getDevice('m4_motor')
        
        # TODO: untill here

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_crazy_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)

