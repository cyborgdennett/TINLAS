from typing import List
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import Range, Image, CameraInfo
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection, Point as _Point # BE CAREFUL this point has x,y while geometry_msgs.Point has x,y,z
from aruco_msgs.msg import MarkerArray, Marker

from .pathplanning import *

import threading
import time
import zmq

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.positioning.motion_commander import MotionCommander
import smol_v2 as com

DEFAULT_HEIGHT = 0.40
MAX_VEL = 0.1

    def ProcessIncomingSwarmMessage(self,dronesDict: dict,dealerSocket,client ):
        for key in dronesDict.keys():
            velocity_x = 0
            velocity_y = 0

            if str(key) not in hardCodedClients:
                continue
            
        # variables recieved from server
            current_pos_X = dronesDict[key]["curPos"][0]
            current_pos_Y = dronesDict[key]["curPos"][1]
            destination_pos_X= dronesDict[key]["targetPos"][0]
            destination_pos_Y= dronesDict[key]["targetPos"][1]
            target = key


        # stopping individual drones if needed
            if rotation_translated_target_x == None and rotation_translated_target_y == None:
                elif str(target) == "3":
                    current_action = com.ActionMoveDrone(3,0,0,None,None,None)
                    target = com.Drone(3)
                    com.DealerSend(dealerSocket,client,target,current_action)
                    continue
                elif str(target) == "7":
                    current_action = com.ActionMoveDrone(7,0,0,None,None,None)
                    target = com.Drone(3)
                    com.DealerSend(dealerSocket,client,target,current_action)
                    continue
                continue

        # execute movement calculations with swapped drone axis
            # Calculating velocity for camera x-axis        
            if current_pos_X < rotation_translated_target_x:
            # logic for moving towards target destination-X pos
                if str(target) == "3" or str(target) == "7":
                    velocity_x = MAX_VEL
                    # print("virtual - ",target,": move to right: y_cmd = ",body_y_cmd)
                else:
                    velocity_y = -1*MAX_VEL
                    # print(target,": move to right: y_cmd = ",body_y_cmd)
            elif current_pos_X > rotation_translated_target_x:
            # logic for moving towards destination-X pos
                if str(target) == "3" or str(target) == "7":
                    velocity_x = -1*MAX_VEL
                    # print("virtual - ",target,": move to left: y_cmd = ",body_y_cmd)
                else:
                    velocity_y = MAX_VEL
                    # print(target,": move to left: y_cmd = ",body_y_cmd)
            elif current_pos_X == rotation_translated_target_x:
            # logic for moving towards destination-X pos
                if str(target) == "3" or str(target) == "7":
                    velocity_x = 0
                    # print("virtual - ",target,": move to left: y_cmd = ",body_y_cmd)
                else:
                    velocity_y = 0
                    # print(target,": move to left: y_cmd = ",body_y_cmd)


            # Calculating velocity for camera y-axis
            if current_pos_Y > rotation_translated_target_y:
            # logic for moving toward destination-Y pos
                if str(target) == "3" or str(target) == "7":
                    velocity_y = MAX_VEL
                    # print("virtual - ",target,": move to front: x_cmd = ", body_x_cmd)
                else:
                    velocity_x = MAX_VEL
                    # print(target,": move to front: x_cmd = ", body_x_cmd)
            elif current_pos_Y < rotation_translated_target_y:
            # logic for moving toward destination-Y pos
                if str(target) == "3" or str(target) == "7":
                    velocity_y = -1*MAX_VEL
                    # print("virtual - ",target,": move to back: x_cmd = ", body_x_cmd)
                else:
                    velocity_x = -1*MAX_VEL
                    # print(targets,": move to back: x_cmd = ", body_x_cmd)  
            elif current_pos_Y == rotation_translated_target_y:
            # logic for hanging still
                if str(target) == "3" or str(target) == "7":
                    velocity_y = 0
                    # print("virtual - ",target,": move to left: y_cmd = ",body_y_cmd)
                else:
                    velocity_x = 0
                    # print(target,": move to left: y_cmd = ",body_y_cmd)


        # writing velocity x-y to correct drone entry
            # Hardcoded swarming support
            if str(target) == str(drone_uris_0_info.client_name):
                    drone_uris_0_target[0] = velocity_x
                    drone_uris_0_target[1] = velocity_y
                    drone_uris_0_hasToMove = True
                    # print("9 body_x_cmd: ", drone_uris_0_target[0]," body_y_cmd: ", drone_uris_0_target[1])
            elif str(target) == str(drone_uris_1_info.client_name):
                    drone_uris_1_target[0] = velocity_x
                    drone_uris_1_target[1] = velocity_y
                    drone_uris_1_hasToMove = True
                    # print("2 body_x_cmd: ", drone_uris_1_target[0]," body_y_cmd: ", drone_uris_1_target[1])
            elif (str(target) == "3"):
                # print("send 3")
                target = com.Drone(3)
                current_action = com.ActionMoveDrone(3,velocity_x,velocity_y,None,None,None)
                com.DealerSend(dealerSocket,client,target,current_action)
                continue
            elif (str(target) == "7"):
                # print("send 7")
                target = com.Drone(7)
                current_action = com.ActionMoveDrone(7,velocity_x,velocity_y,None,None,None)
                com.DealerSend(dealerSocket,client,target,current_action)
                continue
            # print("-----")


                    
def NewMoveDroneAsync(scf,drone_info: com.Drone,drone_target):
        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            print("hello from: ", drone_info.client_name)
            i = 0
            currentMovementX = ""
            currentMovementY = ""
            while True:
                last_x = drone_target[0]
                last_y = drone_target[1]
                time.sleep(0.5)
                i += 5
                if(i >= 4):
                    print("\t\t\t",drone_info.client_name, ": moving with: bdy_x: ",drone_target[0]," bdy_y: ",drone_target[1])

                if last_x == drone_target[0] and last_y == drone_target[1]:
                    print("\t\t\t\t\t currentmovementX ==",currentMovementX)
                    print("\t\t\t\t\t currentmovementY ==",currentMovementY)
                    continue

                if(drone_target[0] == 0 and drone_target[1] == 0 ):
                    mc.stop()
                    pass
                elif(drone_target[0] == None and drone_target[1] == None):
                    print("END OF ROUTINE FOR: ", drone_info.client_name)
                    break
                else:
                    if(drone_target[0] == 0 or drone_target[1] == 0 ):
                        if(i >= 4):
                            print("\t\t\t",drone_info.client_name, ": stop")
                        # mc.stop()
                    
                    if(drone_target[0] > 0):
                        currentMovementX = "forward"
                        if(i >= 4):
                            print("\t\t\t",drone_info.client_name, ": forward")
                        # mc.start_forward(MAX_VEL)
                    elif (drone_target[0] < 0):
                        currentMovementX = "backward"
                        if(i >= 4):
                            print("\t\t\t",drone_info.client_name, ": backward")
                        # mc.start_back(MAX_VEL)

                    if(drone_target[1] > 0):
                        currentMovementY = "left"
                        if(i >= 4):
                            print("\t\t\t",drone_info.client_name, ": left")
                        # mc.start_left(MAX_VEL)
                    elif (drone_target[1] < 0):
                        currentMovementY = "right"
                        if(i >= 4):
                            print("\t\t\t",drone_info.client_name, ": right")
                        # mc.start_right(MAX_VEL)
                    print("linearMotion")
                    mc.start_linear_motion(drone_target[0],drone_target[1],0)
                if(i >= 4):
                    i = 0
                    
class RensPathing(Node)
    def __init__(self):
        super().__init__("RensPathing")
        
        self.drone_pixel_location = dict()
        self.create_subscription(AprilTagDetectionArray, "/fiducial/apriltag_array", self.apriltag_callback, 1)
        
        self.planned_paths = dict()
        
        self.nodes_cmd_vel = dict()
        
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.check_for_new_topics)
        
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
                
                self.get_logger().info("Topic: " + topic[0] + " added id:" + str(marker_id))



    def apriltag_callback(self, msg):
        for detection in msg.detections:
            if detection.id == 6 and len(self.nodes_cmd_vel) == self.agent_count and len(self.planned_paths) is None:
                # path planing
                try:
                    self.planned_paths = path_planning()
                except:
                    pass
                return 
                
            self.drone_pixel_location[detection.id] = \
                (detection.centre.x,detection.centre.y)


if __name__ == '__main__':
    print("in main")
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    thread = threading.Thread(target=RecieveCameraPositionInformation)
    thread.start()
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(NewMoveDroneAsync, args_dict)