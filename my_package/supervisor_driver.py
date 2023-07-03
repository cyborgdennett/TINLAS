import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from math import cos, sin, degrees, radians, pi
import sys
import tf_transformations
from tf2_ros import TransformBroadcaster

# Change this path to your crazyflie-firmware folder
sys.path.append('/home/casper/crazyflie-firmware/build')
import cffirmware

# from controller import Supervisor

class CrazyflieSupervisorDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.timestep = int(self.robot.getBasicTimeStep())
        

        rclpy.init(args=None)
        self.robotName = self.robot.getName()
        # self.namespace = str(self.robotName)
        self.node = rclpy.create_node('cf_supervisor', namespace=self.robotName)
        # self.pub = self.node.create_publisher(String, 'what_to_do', 1)
        self.node.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)
        
        self.node.get_logger().info('Crazyflie supervisor driver initialized for %s' % self.robotName)
        # from controller import Supervisor       
        # self.supervisor = Supervisor()
        self.node.get_logger().info('Crazyflie supervisor driver initialized for %s' % self.robotName)

    def cmd_vel_callback(self, twist):
        self.target_twist = twist
    
    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        

        # dt = self.robot.getTime() - self.past_time
