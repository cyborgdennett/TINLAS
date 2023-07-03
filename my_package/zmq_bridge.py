import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from aruco_msgs.msg import MarkerArray, Marker
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection, Point as _Point
from webots_ros2_msgs.srv import SpawnNodeFromString

import time
import sys
import zmq

from .smol_v2 import * 

from crazyflie_swarm_interfaces.msg import MoveDrone


class ZmqBridge(Node):

    def __init__(self):
        super().__init__('zmq_bridge')        
        
        # ### ROS2 ###
        # create ROS2 publishers
        self.move_drone_publisher = self.create_publisher(MoveDrone, 'move_drone', 1)
        self.move_drone_supervisor_publisher = self.create_publisher(MoveDrone, '/supervisor/move_drone', 1)
        self.detected_drone_location_publisher = self.create_publisher(String, 'detected_drone_location', 1)
        
        self.webots_spawn_service = self.create_client(SpawnNodeFromString, '/spawn_node_from_string')
        self.webots_remove_publisher = self.create_publisher(String, '/remove_node', 1)
        
        # keep a list of simulated drones and real drones
        self.simulated_drones = dict()
        self.real_drones = dict()
        
        # create ROS2 subscribers
        self.create_subscription(AprilTagDetectionArray, '/fiducial/apriltag_array', self.apriltag_callback, 1)
        self.create_subscription(MarkerArray, '/fiducial/aruco_array', self.aruco_callback, 1)
        # set timer for ZMQ publishes
        self.marker_timer = time.time()
        self.marker_timer_treshold = 0.5
        self.apriltag_timer = time.time()
        self.apriltag_timer_treshold = 0.5
        
        # ### ZMQ ###
        # create zmq client
        self.zmq_commander_client = Commander("virtual_commander")
        self.zmq_commander_ip = "tcp://145.24.238.108:5555"
        
        self.zmq_tracker_client = Tracking("virtual_tracking")
        self.zmq_tracker_ip = "tcp://145.24.238.108:5556"
        
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
                str(markerId), 
                int(marker.pose.pose.position.x), 
                int(marker.pose.pose.position.y)
            )
            
            # TODO: not needed at this moment
            continue
            if time.time() - self.marker_timer < self.marker_timer_treshold:
                continue
            
            self.marker_timer = time.time()
            
            DealerSend(self.zmq_tracker_socket, self.zmq_tracker_client, Drone(str(markerId)), action_details)
        pass

    def apriltag_callback(self, msg):
                
        for detection in msg.detections:
            markerId = detection.id
            
            if self.real_drones.get(markerId) != None:
                # no need to send this to the server
                continue
            if self.simulated_drones.get(markerId) == None:
                # add to the list of simulated drones
                self.simulated_drones[markerId] = True
            
                self.get_logger().info('Detected tag with ID: %d' % markerId)
            
            action_details = ActionSendDetectedDroneLocation(
                str(markerId), 
                int(detection.centre.x), 
                int(detection.centre.y)
            )
            
            
            if time.time() - self.apriltag_timer < self.apriltag_timer_treshold:
                continue
            
            self.apriltag_timer = time.time()
            
            DealerSend(self.zmq_tracker_socket, self.zmq_tracker_client, Drone(str(markerId)), action_details)
        
    
    def process_tracker_message(self, message):
        self.get_logger().info('Received message: "%s"' % message.action_details.ACTION_NAME)
        pass
    
    def process_commander_message(self, message):
        
        self.get_logger().info('Received message: "%s"' % message.action_details.ACTION_NAME)

        if message.action_details.ACTION_NAME == ACTION_NAMES.action_move_drone:

            move_drone_msg = MoveDrone() # ROS2 message, the position is a pixel position
            move_drone_msg.pose.position.x = float(0.0 if message.action_details.destination_pos_x == None else message.action_details.destination_pos_x)
            move_drone_msg.pose.position.y = float(0.0 if message.action_details.destination_pos_y == None else message.action_details.destination_pos_y)
            move_drone_msg.pose.position.z = 1.0
            move_drone_msg.pose.orientation.z = float(0.0 if message.action_details.target_rotation == None else message.action_details.target_rotation)

            move_drone_msg.marker = message.action_details.target

            #TODO: needs work, 
            if self.simulated_drones.get(message.action_details.target) == True:
                # only send msg if the drone is simulated
                # TODO fill in cmd_vel msg instead
                self.get_logger().info('Publishing: id:' + str(move_drone_msg.marker) + ' ' + str(move_drone_msg.pose.position.x) + " " + str(move_drone_msg.pose.position.y) + " " + str(move_drone_msg.pose.orientation.z))
                self.move_drone_publisher.publish(move_drone_msg)
                return
            # check if real_drone is not already in the list
            if self.real_drones.get(message.action_details.target) == None:
                self.real_drones[message.action_details.target] = move_drone_msg
            # remove the old location

            # TODO: send msg to crazyflie_supervisor
            self.move_drone_supervisor_publisher.publish(move_drone_msg)

            pass
            _id = message.action_details.target
            delete_node_msg = String()
            delete_node_msg.data = f"twin_{_id}"
            self.webots_remove_publisher.publish(delete_node_msg)
            # update location with webots service
            req = SpawnNodeFromString()
            req.data = f"Crazyflie {{ name \"twin_{_id}\" url \"aruco/aruco_{_id}.jpg\" }}"
            self.webots_spawn_service.call_async(req)
            
        if message.action_details.ACTION_NAME == ACTION_NAMES.action_takeoff_drone:

            pass
        pass
        

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ZmqBridge()


    try:
        rclpy.spin(minimal_publisher)
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_publisher.zmq_commander_socket.close()
        minimal_publisher.zmq_tracker_socket.close()
        minimal_publisher.zmq_context.destroy()
        minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()