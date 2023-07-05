from dataclasses import dataclass, asdict
from typing import List, Optional,Union
import json
import zmq
import datetime
import zmq
import time

# DATACLASSES

@dataclass
class MUTATIONS:
    add: str = "ADD"
    remove: str = "REMOVE"

@dataclass
class ACTION_NAMES:
    action_init_client_to_server_connection: str = "init-client-to-server-connection"
    action_move_drone: str = "move_drone"
    action_send_detected_drone_location: str = "action_send_detected_drone_location"
    action_update_sensor_ouput: str = "action_update_sensor_ouput"
    action_location_target_of_drones: str = "action_location_target_of_drones"
    action_drones_reached_target: str = "action_drones_reached_target"
    action_location_fysical_drones: str = "action_location_fysical_drones"
@dataclass
class Drone:
    client_name: int
    client_position_x: Optional[int] = None
    client_position_y: Optional[int] = None
    client_rotation_z: Optional[float] = None
    client_distance_sensor_UP: Optional[float] = None
    client_distance_sensor_DOWN: Optional[float] = None
    client_distance_sensor_NORTH: Optional[float] = None
    client_distance_sensor_EAST: Optional[float] = None
    client_distance_sensor_SOUTH: Optional[float] = None
    client_distance_sensor_WEST: Optional[float] = None
    CLIENT_TYPE: str = "DRONE"

@dataclass
class Server:
    client_name: str 
    CLIENT_TYPE: str = "SERVER"

@dataclass
class Tracking:
    client_name: str
    CLIENT_TYPE: str = "TRACKING"

@dataclass
class Commander:
    client_name: str
    CLIENT_TYPE: str = "COMMANDER"

@dataclass
class ActionMoveDrone:
    target: int
    target_pos_x: int
    target_pos_y: int
    target_rotation: float
    destination_pos_x: int
    destination_pos_y: int    
    ACTION_NAME: str = ACTION_NAMES.action_move_drone

@dataclass
class ActionInitClientToServerConnection:
    bot_that_initialises_connection: Drone
    mutation: str = MUTATIONS.add
    ACTION_NAME: str = ACTION_NAMES.action_init_client_to_server_connection

#CAMERA -> SERVER
@dataclass
class ActionSendDetectedDroneLocation:
    detected_drone_name: int
    detected_drone_pos_x: int
    detected_drone_pos_y: int
    detected_drone_tvecs: list
    detected_drone_rot_x: float
    detected_drone_rot_y: float
    detected_drone_rot_z: float
    ACTION_NAME: str = ACTION_NAMES.action_send_detected_drone_location

@dataclass
class ActionUpdateSensorOuput:
    drone: Drone
    ACTION_NAME: str = ACTION_NAMES.action_update_sensor_ouput

#DICT FOR FYSICAL DRONES: SERVER -> RENS
@dataclass
class LocationAndTargetOfDrones:
    drones_dict: dict
    ACTION_NAME: str = ACTION_NAMES.action_location_target_of_drones

#UPDATE FOR SIM SERVER -> SIM
@dataclass
class LocationFysicalDrones:
    drone_name: int
    drones_tvec_x: float
    drones_tvec_y: float
    drones_tvec_z: float
    drones_rotation_x: float
    drones_rotation_y: float
    drones_rotation_z: float
    ACTION_NAME: str = ACTION_NAMES.action_location_fysical_drones

@dataclass
class DronesReachedTarget:
    ACTION_NAME: str = ACTION_NAMES.action_drones_reached_target

@dataclass
class Message:
    time_stamp: str
    sender_client_information: Union[Drone, Server]
    action_details: Union[ActionMoveDrone,ActionInitClientToServerConnection,ActionSendDetectedDroneLocation,LocationAndTargetOfDrones,DronesReachedTarget,LocationFysicalDrones]
    message_target: Optional[Union[Drone,int]] = None

# INTERNAL METHODS

def CreateObject(jsonData):
    return deserialize_json(json.loads(jsonData))

# def CreateMessage(Message):
#     messageAsJson = json.dumps(asdict(Message))
#     return messageAsJson

def Receive(socket):
    identity,message = socket.recv_multipart()
    # print("MESSAGE: ",message)
    return CreateObject(message.decode('latin1'))

def GetTime():
    return datetime.datetime.now().strftime("%H:%M:%S") 

def CreateMessage(Message):
    messageAsJson = json.dumps(asdict(Message))
    return messageAsJson

def deserialize_json(json_data):
    # Deserialize the JSON dictionary into objects
    time_stamp = json_data['time_stamp']
    sender_client_information_data = json_data['sender_client_information']
    action_details_data = json_data['action_details']
    message_target_data = json_data.get('message_target')
    # Deserialize sender_client_information into corresponding class
    if sender_client_information_data['CLIENT_TYPE'] == 'DRONE':
        sender_client_information = Drone(**sender_client_information_data)
    elif sender_client_information_data['CLIENT_TYPE'] == 'SERVER':
        sender_client_information = Server(**sender_client_information_data)
    elif sender_client_information_data['CLIENT_TYPE'] == 'TRACKING':
        sender_client_information = Tracking(**sender_client_information_data)
    elif sender_client_information_data['CLIENT_TYPE'] == "COMMANDER":
        sender_client_information = Commander(**sender_client_information_data)
    else:
        print("CLIENTTYPE NOT DETECTED")
        return None
        
    # Deserialize action_details into corresponding class
    if action_details_data['ACTION_NAME'] == ACTION_NAMES.action_move_drone:
        client_details = action_details_data["target"]
        action_details = ActionMoveDrone(**action_details_data)
        # action_details.target = Drone(**client_details) #NEEDS FIX
    elif action_details_data['ACTION_NAME'] == ACTION_NAMES.action_init_client_to_server_connection:
        action_details = ActionInitClientToServerConnection(**action_details_data)
        client_details = action_details_data["bot_that_initialises_connection"]
        action_details.bot_that_initialises_connection = Drone(**client_details)
    elif action_details_data['ACTION_NAME'] == ACTION_NAMES.action_send_detected_drone_location: 
        action_details = ActionSendDetectedDroneLocation(**action_details_data)
    elif action_details_data['ACTION_NAME'] == ACTION_NAMES.action_update_sensor_ouput:
        action_details = ActionUpdateSensorOuput(**action_details_data)
        client_details = action_details_data["drone"]
        action_details.drone = Drone(**client_details)
    elif action_details_data['ACTION_NAME'] == ACTION_NAMES.action_location_target_of_drones:
        action_details = LocationAndTargetOfDrones(**action_details_data)
    elif action_details_data['ACTION_NAME'] == ACTION_NAMES.action_drones_reached_target:
        action_details = DronesReachedTarget(**action_details_data)
    elif action_details_data['ACTION_NAME'] == ACTION_NAMES.action_location_fysical_drones:
        action_details = LocationFysicalDrones(**action_details_data)

    else:
        raise ValueError("Unknown action details type")

    # Deserialize message_target into corresponding class
    message_target = None
    if message_target_data:
        if message_target_data['CLIENT_TYPE'] == 'DRONE':
            message_target = Drone(**message_target_data)
        elif message_target_data['CLIENT_TYPE'] == 'SERVER':
            message_target = Server(**message_target_data)
        elif message_target_data['CLIENT_TYPE'] == 'TRACKING':
            message_target = Tracking(**message_target_data)

    # Create the Message object
    message = Message(time_stamp=time_stamp,
                      sender_client_information=sender_client_information,
                      action_details=action_details,
                      message_target=message_target)

    return message

# PUBLIC METHODS
def PrintErrorIncomingMessage(message):
    print("- - ",GetTime(),"\nERROR: RECIEVED AT - ",message.time_stamp,"\n\t action:\n", message.action_details.ACTION_NAME,"\n\tcontents:\n",message.action_details,"\n\tfrom:\n",message.sender_client_information,"\nEND OF LOG - -")

def PrintLogIncomingMessage(message):
    print("- - ",GetTime(),"\nLOG: RECIEVED AT - ",message.time_stamp,"\n\t action:\n", message.action_details.ACTION_NAME,"\n\tcontents:\n",message.action_details,"\n\tfrom:\n",message.sender_client_information,"\nEND OF LOG - -")

def InitDealerConnection(ip, socket, client):
    #TODO check if incoming socket has DEALER as messaging type       
    ts = GetTime()
    messageClientInfo = client
    messageAction = ActionInitClientToServerConnection(client)
    socket.setsockopt_string(zmq.IDENTITY, client.client_name)  # Set a unique identity for the client using setsockopt_string
    socket.connect(ip)  # bind to the bot socket
    messageTarget = Server("server")
    messageObject = Message(ts, messageClientInfo,messageAction,messageTarget)
    json_message = CreateMessage(messageObject)
    socket.send(json_message.encode())
    time.sleep(0.1)


def SendMoveDrone(socket, client: Union[Drone,Server],message_target, drone_target, current_pos_x, current_pos_y,destination_pos_x,destination_pos_y,current_rotation = None):
    """Sends a message with a ActionMoveDrone message_action. -SCROLL-\n
    For use on servers this returns the action object.\n
    For use on DRONES this uses DealerSend to immediately send the message
    """
    action = ActionMoveDrone(int(drone_target),current_pos_x,current_pos_y,current_rotation,destination_pos_x,destination_pos_y)
    if(client.CLIENT_TYPE == "SERVER"):
        return action
        # RouterSend(socket,client,str(message_target),action)
    if(client.CLIENT_TYPE == "DRONE"):
        DealerSend(socket,client,str(message_target),action)
    pass

def DealerSend(socket, client, target, action):
    ts = GetTime()
    messageObject = Message(ts, client, action, target)
    json_message = CreateMessage(messageObject)
    socket.send(json_message.encode())

def RouterSendAll(socket,client,targetList: list,targetType,action):
    """
    RouterSendAll sends a message to all specified target types using RouterSend.

    socket:\n\t the socket to use
    client:\n\t information object about the sender
    targetList:\n\t a list containing all connected clients
    targetType:\n\t a upper_case string with the target type, i.e. "COMMANDER"
    action:\n\t the action object for the message
    return:\n\t void
    """ 
    for target in targetList:
        if str(target.CLIENT_TYPE) == targetType:
            RouterSend(socket,client,target.client_name,action)

def RouterSend(socket, client, target, action):
    ts = GetTime()
    messageObject = Message(ts, client, action)
    json_message = CreateMessage(messageObject)
    socket.send_multipart([target.encode(), b"", json_message.encode("latin1")])

