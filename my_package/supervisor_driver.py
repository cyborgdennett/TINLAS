import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from crazyflie_swarm_interfaces.msg import MoveDrone

from math import cos, sin, degrees, radians, pi
import re
import sys
import tf_transformations
from tf2_ros import TransformBroadcaster

# Change this path to your crazyflie-firmware folder
sys.path.append('/home/casper/crazyflie-firmware/build')
import cffirmware

# from controller import Supervisor

class CrazyflieSupervisorDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.timestep = int(self.__robot.getBasicTimeStep())
        

        rclpy.init(args=None)
        self.node = rclpy.create_node('cf_supervisor')
        
        self.node.create_subscription(MoveDrone, '/supervisor/move_drone', self.supervisor_move_callback, 10)
        
        self.node.get_logger().info('Crazyflie supervisor driver initialized')

    
    def supervisor_move_callback(self, msg):
        # check if drone is in world
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        roll, pitch, yaw, angle = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        children = self.__robot.getRoot().getMFField('children')
        for i in range(children.getCount()):
            child = children.getMFNode(i)
            if child.getTypeName() != 'CrazyflieNoPhysics':
                continue
            _id = int(re.findall(r'\d+', child.gerName())[0])
            if _id == msg.marker:
                # only need to update
                translation_field = child.getField('translation')
                translation_field.setSFVec3f(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
                rotation_field = child.getField('rotation')
                rotation_field.setSFRotation(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
                return
            
        # drone not found, spawn drone
        
        drone_string = "CrazyflieNoPhysics { name \"twin_agent_" + str(id) + \
            f"\" translation {x} {y} {z} rotation {roll} {pitch} {yaw} {angle}" + "}"
        children.importMFNodeFromString(-1, drone_string)
        
        # removing drone can be done with ROS2Supervisor @'/remove_drone' String() with data = name of drone
    
    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)