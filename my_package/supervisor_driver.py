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

# imports for aruco tag generation
import cv2
import sys
import numpy as np
import os
import tempfile
import time

# from controller import Supervisor

class CrazyflieSupervisorDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.timestep = int(self.__robot.getBasicTimeStep())
        

        rclpy.init(args=None)
        self.node = rclpy.create_node('cf_supervisor')
        
        self.node.create_subscription(MoveDrone, '/supervisor/move_drone', self.move_drone_callback, 10)
        self.node.create_subscription(MoveDrone, '/supervisor/set_goal_tag', self.set_goal_tag_callback, 10)
        self.node.create_subscription(MoveDrone, "/supervisor/chessboard", self.chessboard_callback, 10)
        
        
        # make a cleanup timer to remove stale drones from the world
        self.last_update_time = {}
        self.last_update_treshold = 10.0
        self.cleanup_timer = self.node.create_timer(1.0, self.cleanup_callback)
        
        self.node.get_logger().info('Crazyflie supervisor driver initialized')
        
        # test spawning one
        # md = MoveDrone()    
        # md.marker = 5
        # md.pose.position.x = 0.2
        # md.pose.position.y = -0.2
        # md.pose.position.z = 1.0
        # md.pose.orientation.x = 0.0
        # md.pose.orientation.y = 0.0
        # md.pose.orientation.z = 1.0
        # md.pose.orientation.w = 0.0
        # self.set_goal_tag_callback(md)
        
        # # test spawning an fiducial
        # md.marker = 6
        # md.pose.position.x = 0.25
        # md.pose.position.y = -0.25
        # md.pose.position.z = 0.015
        # md.pose.orientation.z = 0.5
        # self.set_goal_tag_callback(md)
        # md.pose.position.x = 0.1
        # md.pose.position.y = -0.1
        # md.pose.position.z = 0.5
        # md.pose.orientation.z = -0.5
        # md.marker = 7
        # self.set_goal_tag_callback(md)
        # self.move_drone_callback(md)

    def cleanup_callback(self):
        now = time.time()
        for marker, last_update in self.last_update_time.items():
            if now - last_update < self.last_update_treshold:
                continue
            self.node.get_logger().info('Removing stale drone: %i' % marker)
            children = self.__robot.getRoot().getField('children')
            for i in range(children.getCount()):
                child = children.getMFNode(i)
                if child.getTypeName() != 'CrazyflieNoPhysics':
                    continue
                node_name = child.getField('name').getSFString()
                out = re.findall(r'\d+', node_name)
                if len(out) == 0:
                    continue
                _id = int(out[0])
                if _id != marker:
                    continue
                    
                children.removeMF(i)
                # remove from dict
                self.last_update_time.pop(marker)
                # assume only one gets removed
                return
    
    def chessboard_callback(self, msg: MoveDrone):
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        roll, pitch, yaw, angle = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        
        children = self.__robot.getRoot().getField('children')
        for i in range(children.getCount()):
            child = children.getMFNode(i)
            if child.getTypeName() != 'Chessboard':
                continue
            # only need to update
            translation_field = child.getField('translation')
            translation_field.setSFVec3f([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            rotation_field = child.getField('rotation')
            rotation_field.setSFRotation([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            return
            
        # board not found, spawn drone
        
        drone_string = \
        "Chessboard { "+ \
            f"translation {x} {y} {z} " + \
            f"rotation {roll} {pitch} {yaw} {angle} " + \
            f"size 0.5 0.3125 " + \
            f"floorTileSize 0.125 0.125 " + \
        " }"
            
        children.importMFNodeFromString(-1, drone_string)
    
    def move_drone_callback(self, msg: MoveDrone):
        # update timer
        self.last_update_time[msg.marker] = time.time()
        # check if drone is in world
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        roll, pitch, yaw, angle = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        
        children = self.__robot.getRoot().getField('children')
        for i in range(children.getCount()):
            child = children.getMFNode(i)
            if child.getTypeName() != 'CrazyflieNoPhysics':
                continue
            node_name = child.getField('name').getSFString()
            out = re.findall(r'\d+', node_name)
            if len(out) == 0:
                continue
            _id = int(out[0])
            if _id == msg.marker:
                # only need to update
                translation_field = child.getField('translation')
                translation_field.setSFVec3f([x,y,z])
                rotation_field = child.getField('rotation')
                rotation_field.setSFRotation([roll, pitch, yaw, angle])
                return
            
        # drone not found, spawn drone
        self.node.get_logger().info('Spawning drone: %i' % msg.marker)
        self.gen_aruco(msg.marker)
        
        drone_string = \
        "CrazyflieNoPhysics { " + \
            f"name \"twin_agent_{msg.marker}\" " + \
            f"translation {x} {y} {z} " + \
            f"rotation {roll} {pitch} {yaw} {angle} " + \
            "extensionSlot [ " + \
                "CrazyflieFeducial { " +\
                    "url [ " + \
                        f"\"aruco_{msg.marker}.jpg\" " + \
                    "] " + \
                "} "+ \
            "] " + \
        "}"
        children.importMFNodeFromString(-1, drone_string)
        
        # removing drone can be done with ROS2Supervisor @'/remove_drone' String() with data = name of drone
        
    def set_goal_tag_callback(self, msg):
        # check if drone is in world
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        roll, pitch, yaw, angle = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        url = f"aruco_{msg.marker}.jpg"
        
        children = self.__robot.getRoot().getField('children')
        for i in range(children.getCount()):
            child = children.getMFNode(i)
            if child.getTypeName() != 'CrazyflieFeducial':
                continue
            # TODO: needs work
            node_name = child.getField('url').getMFString(-1)
            out = re.findall(r'\d+', node_name)
            if len(out) == 0:
                continue
            _id = int(out[0])
            if _id == msg.marker:
                # only need to update
                
                translation_field = child.getField('translation')
                translation_field.setSFVec3f([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
                rotation_field = child.getField('rotation')
                rotation_field.setSFRotation([0.0,0.0,1.0,0.0])
                # rotation_field.setSFRotation([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
                return
            
        # drone not found, spawn drone
        self.node.get_logger().info('Spawning Fiducial_tag: %i' % msg.marker)
        self.gen_aruco(msg.marker)
        
        #TODO make size variable
        drone_string = "CrazyflieFeducial {" + \
            f"translation {x} {y} {z} " + \
            "size 0.15 0.15 0.001 " + \
            f"url [ \"{url}\" ] " + \
        "}"
            # f"rotation {roll} {pitch} {yaw} {angle} " + \
        children.importMFNodeFromString(-1, drone_string)
        
    # Generate aruco tag to the '/tmp' folder (tested in ubuntu, not sure about windows)   
    def gen_aruco(self, marker_id):
        # check if file exists
        if os.path.exists(tempfile.gettempdir() + f"/aruco_{marker_id}.jpg"):
            return
        
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

        width = 256
        _w = width - int(2 * width/9)
        print(f"width: {width}, _w: {_w}")

        bg = np.ones((width, width, 1), dtype="uint8")
        bg.fill(255)

        tag = np.ones((_w, _w, 1), dtype="uint8")
        cv2.aruco.generateImageMarker(
            arucoDict, marker_id, _w, tag, 1
        )

        x_offset=y_offset = int(width/9)
        # add white border
        bg[y_offset:y_offset+tag.shape[0], x_offset:x_offset+tag.shape[1]] = tag


        f = tempfile.NamedTemporaryFile()
        f.name = tempfile.gettempdir() + f"/aruco_{marker_id}.jpg"

        cv2.imwrite(f.name, bg)
    
    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)