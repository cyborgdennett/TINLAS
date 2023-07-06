import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from aruco_msgs.msg import MarkerArray, Marker
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection, Point as _Point
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist

import time
import re
import sys
import zmq

from .smol_v2 import * 

from crazyflie_swarm_interfaces.msg import MoveDrone

FLIGHT_HEIGHT = 0.3


class ZmqBridge(Node):

    def __init__(self):
        super().__init__('zmq_bridge')        
        
        # ### ROS2 ###
        # create ROS2 publishers
        self.move_drone_publisher = self.create_publisher(MoveDrone, 'move_drone', 1)
        self.move_drone_supervisor_publisher = self.create_publisher(MoveDrone, '/supervisor/move_drone', 1)
        self.set_goal_tag_supervisor_publisher = self.create_publisher(MoveDrone, '/supervisor/set_goal_tag', 1)
        self.detected_drone_location_publisher = self.create_publisher(String, 'detected_drone_location', 1)
        self.GOAL_TAG = 6

        # keep a list of simulated drones and real drones
        self.simulated_drones = dict()
        self.simulated_drones_publisher = dict()
        self.real_drones = dict()
        
        # create ROS2 subscribers
        self.create_subscription(AprilTagDetectionArray, '/fiducial/apriltag_array', self.apriltag_callback, 1)
        self.create_subscription(MarkerArray, '/fiducial/aruco_array', self.aruco_callback, 1)
        # set timer for ZMQ publishes
        self.marker_timer = time.time()
        self.marker_timer_treshold = 0.5
        self.apriltag_timer = time.time()
        self.apriltag_timer_treshold = 0.2
        
        # ### ZMQ ###
        # create zmq client
        self.zmq_commander_client = Commander("virtual_commander")
        self.zmq_commander_ip = "tcp://145.24.238.26:5555"
        
        self.zmq_tracker_client = Tracking("virtual_tracking")
        self.zmq_tracker_ip = "tcp://145.24.238.26:5556"
        
        # create sockets
        self.zmq_context = zmq.Context()
        # self.zmq_commander_context = zmq.Context()
        # self.zmq_commander_socket = self.zmq_commander_context.socket(zmq.DEALER)
        self.zmq_commander_socket = self.zmq_context.socket(zmq.DEALER)
        
        # self.zmq_tracker_context = zmq.Context()
        # self.zmq_tracker_socket = self.zmq_tracker_context.socket(zmq.DEALER)
        self.zmq_tracker_socket = self.zmq_context.socket(zmq.DEALER)
        
        # init connections
        InitDealerConnection(self.zmq_commander_ip, self.zmq_commander_socket, self.zmq_commander_client)
        InitDealerConnection(self.zmq_tracker_ip, self.zmq_tracker_socket, self.zmq_tracker_client)
        
        # create poller
        # Create the poller
        self.poller = zmq.Poller()
        self.poller.register(self.zmq_commander_socket, zmq.POLLIN)  # Register the socket for input events
        self.poller.register(self.zmq_tracker_socket, zmq.POLLIN)
        
        # create timer for poller
        self.create_timer(0.1, self.poller_callback)
        
        self.get_logger().info('ZMQ Initialized')

    def poller_callback(self):
    
        socks = dict(self.poller.poll(timeout=10))  # Poll for events

        if self.zmq_tracker_socket in socks and socks[self.zmq_tracker_socket] == zmq.POLLIN:
            message = Receive(self.zmq_tracker_socket)  # Receive the message from socket1
            # Process the message from socket1
            self.get_logger().info('Msg recv tracker')
            self.process_tracker_message(message)

        if self.zmq_commander_socket in socks and socks[self.zmq_commander_socket] == zmq.POLLIN:
            message = Receive(self.zmq_commander_socket)  # Receive the message from socket2
            # Process the message from socket2
            self.get_logger().info('Msg recv commander')
            self.process_commander_message(message)
            
        # continue
        return
        
        #TODO: use poller
        message = Receive(self.zmq_commander_socket)
        if message.action_details.ACTION_NAME == ACTION_NAMES.action_move_drone:
            # target: int
            # target_pos_x: int
            # target_pos_y: int
            # target_rotation: float
            # destination_pos_x: int
            # destination_pos_y: int
                
            move_drone_msg = ActionMoveDrone()
            move_drone_msg.pose.position.x = float(0.0 if message.action_details.destination_pos_x == None else message.action_details.destination_pos_x)
            move_drone_msg.pose.position.y = float(0.0 if message.action_details.destination_pos_y == None else message.action_details.destination_pos_y)
            move_drone_msg.pose.position.z = 1.0
            move_drone_msg.pose.orientation.z = float(0.0 if message.action_details.target_rotation == None else message.action_details.target_rotation)
            
            self.get_logger().info('Publishing: ' + str(move_drone_msg.pose.position.x) + " " + str(move_drone_msg.pose.position.y) + " " + str(move_drone_msg.pose.orientation.z))
            
            self.move_drone_publisher.publish(move_drone_msg)
            
            # continue
            return
            
            current_pos_X = message.action_details.target_pos_x
            current_pos_Y = message.action_details.target_pos_y
            current_rotation = message.action_details.target_rotation
            destination_pos_X = message.action_details.destination_pos_x
            destination_pos_Y = message.action_details.destination_pos_y
            print(str(message.action_details.target),": current_pos_x: ", current_pos_X,"\t | current_pos_y: ",current_pos_Y)
            print(str(message.action_details.target),": target_pos__x: ", destination_pos_X,"\t | target_pos__y: ",destination_pos_Y)

            rotation_translated_target_x = destination_pos_X# target_pos_X*math.cos(current_rotation)-target_pos_Y*math.sin(current_rotation)
            rotation_translated_target_y = destination_pos_Y# target_pos_X*math.sin(current_rotation)+target_pos_Y*math.cos(current_rotation)

            # time.sleep(0.2)
            if rotation_translated_target_x == None or rotation_translated_target_y == None:
                hasToMove = False
                print(str(message.action_details.target),": recieved None")
                # continue
                return
            else:
                if current_pos_X > rotation_translated_target_x:
                    # Implement logic for moving towards target X pos
                    body_y_cmd = MAX_VEL
                    hasToMove = True
                    print(str(message.action_details.target),": move to back: y_cmd = ",body_y_cmd)
                elif current_pos_X < rotation_translated_target_x:
                    # Implement logic for moving towards target X pos
                    body_y_cmd = -1*MAX_VEL
                    hasToMove = True
                    print(str(message.action_details.target),": move to front: y_cmd = ",body_y_cmd)
                if current_pos_Y > rotation_translated_target_y:
                    # Implement logic for moving toward target Y pos
                    body_x_cmd = MAX_VEL
                    hasToMove = True
                    print(str(message.action_details.target),": move right: x_cmd = ", body_x_cmd)
                elif current_pos_Y < rotation_translated_target_y:
                    # Implement logic for moving toward target Y pos
                    body_x_cmd = -1*MAX_VEL
                    hasToMove = True
                    print(str(message.action_details.target),": move left: x_cmd = ", body_x_cmd)  
                if hasToMove == False:
                    body_y_cmd = 0 
                    body_x_cmd = 0
                    print(str(message.action_details.target),": STOP!: ",body_x_cmd," ",body_y_cmd)
            # Hardcoded swarming support
            if str(message.action_details.target) == drone_uris_0_info.client_name:
                # implement logic for uri0
                if hasToMove:
                    drone_uris_0_target[0] = body_x_cmd
                    drone_uris_0_target[1] = body_y_cmd
                    drone_uris_0_hasToMove = True
                    pass
                else:
                    drone_uris_0_target[0] = 0
                    drone_uris_0_target[1] = 0
                    drone_uris_0_hasToMove = False
                    pass
                pass
            elif str(message.action_details.target) == drone_uris_1_info.client_name:
                # implement logic for uri1
                if hasToMove:
                    drone_uris_1_target[0] = body_x_cmd
                    drone_uris_1_target[1] = body_y_cmd
                    drone_uris_1_hasToMove = True
                    pass
                else:
                    drone_uris_1_target[0] = 0
                    drone_uris_1_target[1] = 0
                    drone_uris_1_hasToMove = False
                    pass
                pass
    
        
    def aruco_callback(self, msg):
        for marker in msg.markers:
            markerId = marker.id
            
            if self.real_drones.get(markerId) != None:
                # no need to send this to the server
                continue
            if self.simulated_drones.get(markerId) == None:
                # add to the list of simulated drones
                self.simulated_drones[markerId] = True
            
                self.get_logger().info('Detected tag with ID: %d' % markerId)
            #TODO send the right message
            action_details = ActionSendDetectedDroneLocation(
                markerId, 
                0,
                0,
                [marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z],
                marker.pose.pose.orientation.x,
                marker.pose.pose.orientation.y,
                marker.pose.pose.orientation.z,
            )
            """detected_drone_name: str
                detected_drone_pos_x: int
                detected_drone_pos_y: int
                detected_drone_tvecs: list
                detected_drone_rot_x: float
                detected_drone_rot_y: float
                detected_drone_rot_z: float"""
            
            # TODO: not needed at this moment
            continue
            if time.time() - self.marker_timer < self.marker_timer_treshold:
                continue
            
            self.marker_timer = time.time()
            
            DealerSend(self.zmq_tracker_socket, self.zmq_tracker_client, Drone(str(markerId)), action_details)
        pass

    def apriltag_callback(self, msg):
                
        if time.time() - self.apriltag_timer < self.apriltag_timer_treshold:
            return

        self.apriltag_timer = time.time()

        for detection in msg.detections:
            markerId = detection.id

            if self.real_drones.get(markerId) != None:
                # no need to send this to the server
                continue
            if self.simulated_drones.get(markerId) == None:
                # add to the list of simulated drones
                self.simulated_drones[markerId] = (-1,-1)
                        
            new_pos = (int(detection.centre.x/10), int(detection.centre.y/20))
            if self.simulated_drones[markerId] == new_pos:
                continue
            self.simulated_drones[markerId] = new_pos
            if markerId == 0:
                continue
            action_details = ActionSendDetectedDroneLocation(
                markerId, 
                int(detection.centre.x/10), 
                int(detection.centre.y/20),
                list(),
                0.0,
                0.0,
                0.0,
            )

            DealerSend(self.zmq_tracker_socket, self.zmq_tracker_client, Drone(str(markerId)), action_details)


    def process_tracker_message(self, message):
        self.get_logger().info('Received message: "%s"' % message.action_details.ACTION_NAME)
        pass
    
    def process_commander_message(self, message):
        
        self.get_logger().info('Received message: "%s"' % message.action_details.ACTION_NAME)

        if message.action_details.ACTION_NAME == ACTION_NAMES.action_location_fysical_drones:
            if self.real_drones.get(message.action_details.drone_name) == None:
                self.real_drones[message.action_details.drone_name] = True
            self.get_logger().info('Received message: "%s"' % str(message.action_details))
            move_simulated_drone_msg = MoveDrone()
            move_simulated_drone_msg.marker = message.action_details.drone_name
            # move_simulated_drone_msg.pose.position.x = float(0.0 if message.action_details.destination_pos_x == None else message.action_details.destination_pos_x)
            move_simulated_drone_msg.pose.position.x = message.action_details.drones_tvec_x
            move_simulated_drone_msg.pose.position.y = message.action_details.drones_tvec_y
            move_simulated_drone_msg.pose.position.z = FLIGHT_HEIGHT  # message.action_details.drones_tvec_z # 0.7 # use 0.7 if this spawn under map
            # move_simulated_drone_msg.pose.orientation.x = float(message.action_details.drones_rotation_x)
            # move_simulated_drone_msg.pose.orientation.y = float(message.action_details.drones_rotation_y)
            move_simulated_drone_msg.pose.orientation.z = 1.0
            # move_simulated_drone_msg.pose.orientation.w = float(message.action_details.drones_rotation_z)
            
            # send to the supervisor

            if message.action_details.drone_name == self.GOAL_TAG: 
                move_simulated_drone_msg.pose.position.z = 0.001
                self.set_goal_tag_supervisor_publisher.publish(move_simulated_drone_msg)
            else:
                self.move_drone_supervisor_publisher.publish(move_simulated_drone_msg)

        if message.action_details.ACTION_NAME == ACTION_NAMES.action_move_drone:

            move_drone_msg = MoveDrone() # ROS2 message, the position is a pixel position, only have to listen to the x and y position
            move_drone_msg.marker = message.action_details.target
            
            self.get_logger().info('Received message: "%s"' % str(message.action_details))

            # if self.simulated_drones.get(message.action_details.target) == True:
            move_drone_msg.pose.position.x = float(0.0 if message.action_details.target_pos_x == None else message.action_details.target_pos_x)
            move_drone_msg.pose.position.y = float(0.0 if message.action_details.target_pos_y == None else message.action_details.target_pos_y)
            move_drone_msg.pose.position.z = 1.0
            move_drone_msg.pose.orientation.z = float(0.0 if message.action_details.target_rotation == None else message.action_details.target_rotation)

            # only send msg if the drone is simulated
            # self.move_drone_publisher.publish(move_drone_msg)

            # make move command
            twist = Twist()
            twist.linear.x = float(0.0 if message.action_details.target_pos_x == None else message.action_details.target_pos_x)
            twist.linear.y = float(0.0 if message.action_details.target_pos_y == None else message.action_details.target_pos_y)
            
            # SPEED LIMITING
            maxspeed = 0.05
            twist.linear.x = maxspeed if twist.linear.x > maxspeed else twist.linear.x
            twist.linear.y = maxspeed if twist.linear.y > maxspeed else twist.linear.y
            self.get_logger().info('Publishing: id:' + str(move_drone_msg.marker) + ' ' + str(twist.linear.x) + ' ' + str(twist.linear.y))
            
            # the following is for move_linear
            if self.simulated_drones_publisher.get(move_drone_msg.marker) is None:
                for topic in self.get_topic_names_and_types():
                    if not (topic[0].startswith("/agent_") and topic[0].endswith("/cmd_vel")):
                        continue
                
                    marker_id = int(re.findall(r'\d+', topic[0])[0])
                    if not (marker_id == message.action_details.target):
                        continue
                    
                    self.simulated_drones_publisher[move_drone_msg.marker] = self.create_publisher(Twist, topic[0], 10)

                if self.simulated_drones_publisher[move_drone_msg.marker] == None:                        
                    # no topic found, return
                    return
                
            # publish message 
            self.simulated_drones_publisher[move_drone_msg.marker].publish(twist)


def main(args=None):
    rclpy.init(args=args)

    zmq_bridge = ZmqBridge()


    try:
        rclpy.spin(zmq_bridge)
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        zmq_bridge.zmq_commander_socket.close()
        zmq_bridge.zmq_tracker_socket.close()
        zmq_bridge.zmq_context.destroy()
        zmq_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()