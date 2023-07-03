import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection, Point

import sys
import zmq

from .smol_v2 import * 

from crazyflie_swarm_interfaces.msg import MoveDrone


class ZmqBridge(Node):

    def __init__(self):
        super().__init__('zmq_bridge')        
        
        # create ROS2 publishers
        self.move_drone_publisher = self.create_publisher(MoveDrone, 'move_drone', 10)
        self.detected_drone_location_publisher = self.create_publisher(String, 'detected_drone_location', 10)
        
        
        # create ROS2 subscribers
        self.create_subscription(AprilTagDetectionArray, '/fiducial/apriltag_array', self.fiducial_callback, 10)
        
        
        
        
        self.mqz_client_commander = Commander("Tracking")
        self.mqz_router_ip = "tcp://145.24.238.11:5555"

        # create zmq socket
        self.zmq_context = zmq.Context()
        self.dealerSocket = self.zmq_context.socket(zmq.DEALER)
        self.client = self.mqz_client_commander
        self.ip = self.mqz_router_ip
        InitDealerConnection(self.ip, self.dealerSocket, self.client)


        while True:
            
            #TODO: use poller
            message = Receive(self.dealerSocket)
            if message.action_details.ACTION_NAME == ACTION_NAMES.action_move_drone:
                # target: int
                # target_pos_x: int
                # target_pos_y: int
                # target_rotation: float
                # destination_pos_x: int
                # destination_pos_y: int
                    
                move_drone_msg = MoveDrone()
                move_drone_msg.pose.position.x = float(0.0 if message.action_details.destination_pos_x == None else message.action_details.destination_pos_x)
                move_drone_msg.pose.position.y = float(0.0 if message.action_details.destination_pos_y == None else message.action_details.destination_pos_y)
                move_drone_msg.pose.position.z = 1.0
                move_drone_msg.pose.orientation.z = float(0.0 if message.action_details.target_rotation == None else message.action_details.target_rotation)
                
                self.get_logger().info('Publishing: ' + str(move_drone_msg.pose.position.x) + " " + str(move_drone_msg.pose.position.y) + " " + str(move_drone_msg.pose.orientation.z))
                
                self.move_drone_publisher.publish(move_drone_msg)
                
                continue
                
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
                    continue
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

    def fiducial_callback(self, msg):
        
        for detection in msg.detections:
            markerId = detection.id
            
            action_details = ActionSendDetectedDroneLocation(
                str(markerId), 
                int(detection.centre.x), 
                int(detection.centre.y)
            )
            
            DealerSend(self.dealerSocket, self.mqz_client_commander, Drone(str(markerId)), action_details)
        
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
        minimal_publisher.zmq_context.destroy()
        minimal_publisher.dealerSocket.close()
        minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()