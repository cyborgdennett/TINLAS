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

import copy



class DroneStatus(Enum):
    IDLE = 0
    MOVING = 1
    ARRIVED = 2
    HOVER = 3


@dataclass
class Drone:
    id: int = -1
    position: Point = Point()
    orientation: Quaternion  = Quaternion()
    last_seen: float = time.time()
    target: Point | None = None
    twist: Twist = Twist()
    cmd_vel: str = ""
    status: DroneStatus = DroneStatus.IDLE
    pixel_position: _Point = None
    fiducial_offset: float = 0.0
    target_found: bool = False
    


class Swarm:
    def __init__(self, logger):
        self.__drones = dict()
        self.logger = logger
        
    @property
    def drones(self):
        return self.__drones.values()

    def add_drone(self, id):
        drone = Drone(id)
        drone.cmd_vel = f"agent_{id}/cmd_vel"
        drone.last_seen = time.time()
        
        self.__drones[id] = drone
    
    def update_position(self, id, position):
        if self.__drones.get(id) is None:
            self.add_drone(id)
        self.__drones[id].position = position
        self.__drones[id].last_seen = time.time()

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
        
    def get_drones(self):
        return self.__drones
        
    def get_drone(self, id) -> Drone | None:
        return self.__drones.get(id)

class SwarmPathing(Node):
    def __init__(self):
        super().__init__("swarm_pathing")
        
        self.swarm = Swarm(self.get_logger())
        
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
        self.gps_list = dict()
        # Create a timer that fires every second to check for new nodes
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.check_for_new_topics)
        
        execution_period = .5 # around 3 webots ticks
        self.execute_timer = self.create_timer(execution_period, self.execute)
        self.busy = False
        
        self.swarm_executor: SwarmExecutor | None = None
        
        self.get_logger().info("SwarmPathing initialized")
        
    def execute(self):
        # self.__selected_swarm_goal().
        if self.busy:
            return
        if self.swarm_executor is None and len(self.swarm.get_drones()) >= 2:
            self.swarm_executor = StraightToTargetSwarmExecutor(self.swarm)
            return
        if self.swarm.get_drone(1) is not None and self.swarm.get_drone(1).position.x == 0:
            return
        if self.swarm_executor is None:
            return
        # self.get_logger().info("Executing")
        self.busy = True
        # self.__move_swarm_forward()
        self.swarm_executor.execute()
        
        # send twist to all drones 
        for drone in self.swarm.drones:
            self.nodes_cmd_vel[drone.id].publish(drone.twist)
        self.busy = False
        
    def aruco_callback(self, msg):
        # TODO: once aruco is fixed, turn on again
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
    
    def gps_callback(self, msg, marker):
        return
        # self.get_logger().info(f"GPS callback {marker} {msg}")
        # self.get_logger().info(f"GPS callback class: {msg.__class__}")
        self.swarm.update_position(marker, msg)

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
                
                self.get_logger().info("Topic: " + topic[0] + " added id:" + str(marker_id))
            # self.get_logger().info("Topic: " + topic[0] + " not added")
            # listen to gps
            if topic[0].startswith("/agent") and topic[0].endswith("/gps"):
                marker_id = int(re.findall(r'\d+', topic[0])[0])
                if self.gps_list.get(marker_id) is not None:
                    continue

                t = marker_id
                self.gps_list[marker_id] = self.create_subscription(
                    Point,
                    topic[0],
                    lambda msg: self.gps_callback(msg, t),
                    1,
                )
                
                self.get_logger().info("GPS: " + topic[0] + " added")
                
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
            drone.twist.linear.y = 0.1
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
            q = copy.deepcopy(drone.orientation)
            # degrees to radians
            q.x = np.radians(q.x)
            q.y = np.radians(q.y)
            q.z = np.radians(q.z)

            # calculate the fiducial yaw
            yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
            yaw = yaw - np.radians(90) # test if this is correct
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
            drone.twist.angular.z = 0.0 # This could be used to change yaw.

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
    def execute(self): # updates the twist to be sent to the drones
        raise NotImplemented

class StraightToTargetSwarmExecutor(SwarmExecutor):
    def __init__(self, swarm):
        super().__init__(swarm)
        # set a point for each drone to go to
        self.targets = [[0.0,-0.1],[0.0,0.2],[-.2,.0],[-.2,-.2],[.2,-.2]]
        self.target_index = 0
        self.precision = 0.005
        self.maxspeed = 0.05  # keep it slow
        self.slowdown_distance = 0.1 # distance to target where the drone starts to slow down
        # for i, drone in enumerate(self.swarm.drones):
        #     drone.target = (targets[i][0],targets[i][1],.7)
        #     self.target_index += 1
    """ if the drone moves x + 0.1 , it goes to the east(right), if north is top of the screen"""
    """ if the drone moves y + 0.1 , it goes to the north(forward), if north is top of the screen"""
    """ 90 degree to the right(with the clock) = -1.5708 in webots"""
    """ 90 degree to the left(against the clock) = 1.5708 in webots"""
    def execute(self):
        for drone in self.swarm.drones:
            if drone.target is None:
                self.swarm.logger.info("Drone " + str(drone.id) + " setting target " + str(self.targets[self.target_index]) + " Position "+ str(drone.position))
                drone.target = Point()
                drone.target.x = self.targets[self.target_index][0]
                drone.target.y = self.targets[self.target_index][1]
                self.target_index += 1
            # if close enough to target, set to hover
            if abs(drone.target.x  - drone.position.x) < self.precision and abs(drone.target.y  - drone.position.y) < self.precision:
                drone.twist = Twist()
                drone.twist.linear.x = 0.0
                drone.twist.linear.y = 0.0
                
                # let the drone turn in circles to show it has completed the task
                drone.twist.angular.z = 1.5
                if not drone.target_found:
                    self.swarm.logger.info("Drone " + str(drone.id) + " reached target " + str(drone.target) + " Position "+ str(drone.position))
                    drone.target_found = True
                continue
            
            drone.target_found = False
            
            # find angle to target
            angle_to_target = atan2(drone.target.y - drone.position.y , drone.target.x - drone.position.x)

            # degrees to radians the fiducial yaw            
            yaw = np.radians(drone.orientation.z + drone.fiducial_offset) #have to do -90 because the fiducial is placed wrong
            # yaw = yaw + np.radians(90) # test if this is correct
            # find the difference between orientation and angle to radius based on the yaw
            new_yaw = angle_to_target-yaw

            # calculate the distance to the target
            d = sqrt((drone.position.x - drone.target.x)**2 + (drone.position.y - drone.target.y)**2)
            # translate to linear x, y
            # get the speed based on the distance to the target
            speed = self.maxspeed
            if d < self.slowdown_distance:
                speed = speed * abs(d) / self.slowdown_distance
            
            drone.twist = Twist()
            drone.twist.linear.x = speed*sin(new_yaw) # idk why this is sin and not cos, in the formula it is
            drone.twist.linear.y = speed*cos(new_yaw)
            drone.twist.angular.z = 0.0 # This could be used to change yaw.
            
            
            # self.swarm.logger.info("id: "+str(drone.id)+\
            #     " \tx= "+ str(drone.position.x)+\
            #     " \ty= "+ str(drone.position.y)+\
            #     " \ttx= "+ str(drone.target.x)+\
            #     " \tty= "+ str(drone.target.y)+\
            #     " \td= "+str(d)+\
            #     " \tangle_to_target= "+str(np.degrees(angle_to_target))+\
            #     " \tyaw= "+str(np.degrees(yaw))+\
            #     " \tnew_yaw= "+str(np.degrees(new_yaw))+\
            #     " \ttw_x= "+str(drone.twist.linear.x)+\
            #     " \ttw_y= "+str(drone.twist.linear.y)+\
            #     " \ttw_z= "+str(drone.twist.linear.z))
            
            
        

class StraightlineFormationSwarmExecutor(SwarmExecutor):
    def __init__(self, swarm):
        super().__init__(swarm)
        # make a straightline
        self.targets = [
            [self.swarm.camera_pixels[0]/2, self.swarm.camera_pixels[1] / len(self.swarm.drones + 2) * (i + 1) ] for i in range(len(self.swarm.drones))
        ]
        self.swarm.logger("StraightlineSwarmExecutor targets: " + str(self.targets))
        
        self.precision = 5 # how close to the target is close enough
        
        # Allocate targets to drones
        # Could be more efficient, (for example by pathing the closest drone to the closest target) but this is fine for now
        for i, drone in enumerate(self.swarm.drones):
            drone.target = (self.targets[i][0],self.targets[i][1],1)
        
    def execute(self):
        # make a flight path for each drone
        # hover points will be like an obstacle
        hover = []
        start = []
        end = []
        for drone in self.swarm.drones:
            # if close enough to target, set to hover
            if abs(drone.target.x  - drone.position.x) < self.precision and abs(drone.target.y  - drone.position.y) < self.precision:
                hover.append([drone.id, drone.position.x, drone.position.y, 1])
                continue
            # TODO.
            

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
