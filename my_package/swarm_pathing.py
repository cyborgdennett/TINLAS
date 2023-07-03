import time
import re
from enum import Enum

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range, Image, CameraInfo
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection, Point as _Point # BE CAREFUL this point has x,y while geometry_msgs.Point has x,y,z
from aruco_msgs.msg import MarkerArray, Marker

import numpy as np
from math import pi, atan2, sqrt, sin, cos, acos, asin, atan
from dataclasses import dataclass
from typing import Tuple




class DroneStatus(Enum):
    IDLE = 0
    MOVING = 1
    ARRIVED = 2
    HOVER = 3


@dataclass
class Drone:
    id: int = -1
    position: Point | None = None
    orientation: Quaternion | None = None
    last_seen: float = time.time()
    target: Point = (0, 0, 0)
    twist: Twist = Twist()
    cmd_vel: str = ""
    status: DroneStatus = DroneStatus.IDLE
    pixel_position: _Point = None
    


class Swarm:
    def __init__(self):
        self.__drones = dict()
        
    @property
    def drones(self):
        return self.__drones.values()

    def add_drone(self, id):
        drone = Drone(id)
        drone.cmd_vel = f"agent_{id}/cmd_vel"
        drone.last_seen = time.time()
        
        self.__drones[id] = drone

    def update_drone(self, id, position, orientation):
        self.__drones[id].position = position
        self.__drones[id].orientation = orientation
        self.__drones[id].last_seen = time.time()
    
    def update_pixel_position(self, id, point):
        self.__drones[id].pixel_position = point
    
    def update_twist(self, id, twist):
        self.__drones[id].movement = twist
        
    def set_goal(self, id, goal):
        self.__drones[id].target = goal
        
    def get_drone(self, id) -> Drone | None:
        return self.__drones.get(id)

class SwarmPathing(Node):
    def __init__(self):
        super().__init__("swarm_pathing")
        
        self.swarm = Swarm()
        
        self.camera_pixels = [1000,1000]

        # self.__selected_swarm_goal = self.__empty_func
        self.__selected_swarm_goal = self.__circle
        self.__set_swarm_goal_data = String()
        self.create_subscription(
            String, "/set_swarm_goal", self.__set_swarm_goal_callback, 1
        )

        

        # subscribe to pixel data of the aruco markers
        self.camera_pixel_data = dict()
        self.create_subscription(AprilTagDetectionArray, "/fiducial/apriltag_array", self.apriltag_callback, 1)
        
        # subscribe to aruco position and orientation
        self.aruco_marker = dict()
        self.create_subscription(MarkerArray, "/fiducial/aruco_array", self.aruco_callback, 1)


        self.nodes_cmd_vel = dict()
        # Create a timer that fires every second to check for new nodes
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.check_for_new_topics)
        
        execution_period = 0.1 # around 3 webots ticks
        self.execute_timer = self.create_timer(0.1, self.execute)
        

        self.get_logger().info("SwarmPathing initialized")
        
    def execute(self):
        self.__selected_swarm_goal()
        
    def aruco_callback(self, msg):
        for marker in msg.markers:
            self.aruco_marker[marker.id] = marker
            if self.swarm.get_drone(marker.id) is None:
                continue    

            self.swarm.update_drone(marker.id, marker.pose.pose.position, marker.pose.pose.orientation)


    def apriltag_callback(self, msg):
        for detection in msg.detections:
            self.camera_pixel_data[detection.id] = detection

            if self.swarm.get_drone(detection.id) is None:
                continue

            # update drone position
            self.swarm.update_pixel_position(detection.id, detection.centre)            

    def check_for_new_topics(self):
        # self.get_logger().info("Checking for new topics") 
        topics = self.get_topic_names_and_types()
        for topic in topics:
            # check if already known
            # check if topic starts with /agent 
            if topic[0].startswith("/agent") and topic[0].endswith("/cmd_vel"):
                # get id from topic ex: /agent_1/cmd_vel = 1
                marker_id = int(re.findall(r'\d+', topic[0])[0])
                if self.nodes_cmd_vel.get(marker_id) is not None:
                    continue
                
                self.nodes_cmd_vel[marker_id] = self.create_publisher(Twist, topic[0], 1)
                self.swarm.add_drone(marker_id)
                
                self.get_logger().info("Topic: " + topic[0] + " added")
                
            # TODO: subscribe to drone specific topics such as IR, camera, etc.
            # use following code to manage multiple subscriptions
            # t = topic[0]
            # self.nodes_cmd_vel[
            #         topic[0]
            #     ] = self.create_subscription(
            #         Point,
            #         topic[0],
            #         lambda msg: self.__fiducial_callback(msg, t),
            #         1,
            #     )
            

        # check if anything went offline
        # TODO: won't work untill you stop listening to topics
        # for topic in self.nodes_cmd_vel.keys():
        #     if topic not in [i[0] for i in topics]:
        #         self.nodes_cmd_vel[topic].destroy()
        #         self.nodes_cmd_vel.pop(topic)
        #         self.get_logger().info("Topic " + str(topic) + " went offline")
        #         return

    def __fiducial_callback(self, message, topic):
        # check which drone sent the message
        self.__camera_position_data[topic] = message

    def __set_swarm_goal_callback(self, message):
        # create function dictionary
        __swarm_goal_dict = {
            "straightline": self.__straightline,
            "circle": self.__circle,
        }

        if message.data in __swarm_goal_dict.keys():
            self.__selected_swarm_goal = __swarm_goal_dict[message.data]

    def set_goal(self, markerID, cX, cY):
        pass
    
    def __empty_func(self):
        pass
    
    def __move_swarm_forward(self):
        for drone in self.swarm.drones:
            drone.twist = Twist()
            drone.twist.linear.x = 0.1
            self.nodes_cmd_vel[drone.id].publish(drone.twist)
            
    def __circle(self):
        l = len(self.swarm.drones)
        if l == 0:
            return
        # set center of circle
        center = (self.camera_pixels[0]/2, self.camera_pixels[1]/2)
        # set radius of circle to the smallest of the two
        r = center[0]/2 if center[0] < center[1] else center[1]/2
        # set the motion (clockwise 1 or counterclockwise -1)
        m = 1
        # TODO: set the angle between the drones, use this for space between drones
        da = np.radians(360/l-180)
        # find for each drone a straight line to the radius
        for drone in self.swarm.drones:
            if drone.pixel_position is None or drone.orientation is None:
                continue
            # distance to centre
            d = sqrt((drone.pixel_position.x - center[0])**2 + (drone.pixel_position.y - center[1])**2)
            # r is known, d is known, find angle to radius
            angle_to_center = atan2(center[1] - drone.pixel_position.y , center[0] - drone.pixel_position.x)
            # set to the clockwise or counterclockwise motion
            p = point_on_radius = (center[0] + r*cos(angle_to_center) * m, center[1] + r*sin(angle_to_center) * m)            
            a = angle_to_radius = atan2(point_on_radius[1] - drone.pixel_position.y , point_on_radius[0] - drone.pixel_position.x)
            # get the quaternion of the drone, also have to know how the fiducial is located on the drone
            q = drone.orientation
            # degrees to radians
            q.x = np.radians(q.x)
            q.y = np.radians(q.y)
            q.z = np.radians(q.z)
            
            # calculate the fiducial yaw
            yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
            # find the difference between orientation and angle to radius based on the yaw
            dyaw = a-yaw
            new_yaw = yaw + dyaw
            
            # translate to linear x, y
            maxspeed = 0.05  # keep it slow
            x = maxspeed*cos(new_yaw)
            y = -maxspeed*sin(new_yaw)
            
            drone.twist = Twist()
            drone.twist.linear.x = x
            drone.twist.linear.y = y
            drone.twist.linear.z = 1.0  # keep it at 1 meter
            
            # self.get_logger().info("Drone " + str(drone.id) + " yaw: " + str(yaw) + " dyaw: " + str(dyaw) + " new_yaw: " + str(new_yaw) + " x: " + str(x) + " y: " + str(y) + " z: " + str(drone.twist.linear.z) + " d: " + str(d) + " a: " + str(a) + " da: " + str(da) + " p: " + str(p) + " angle_to_center " + str(angle_to_center) + " center " + str(center) + " point_on_radius " + str(point_on_radius))
        
            # send msg to drone
            self.nodes_cmd_vel[drone.id].publish(drone.twist)

    # TODO: This function should put all the drones in a straight line, and then move them forward    
    def __straightline(self, markerID, cX, cY):
        # set target pos to:
        arr = [
            [10, 25],
            [20, 25],
            [30, 25],
            [40, 25],
            [50, 25],
            [60, 25],
            [70, 25],
            [80, 25],
            [90, 25],
            [100, 25],
        ]
        # check if position is reached +- 1 coord

        # set next target pos to:
        return arr


class SwarmExecutor:
    def __init__(self, swarm):
        self.swarm = swarm
    def execute(self):
        raise NotImplemented

def main(args=None):
    rclpy.init(args=args)
    swarm_pathing = SwarmPathing()
    
    rclpy.spin(swarm_pathing)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    swarm_pathing.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
