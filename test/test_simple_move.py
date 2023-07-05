from geometry_msgs.msg import Point, Twist, Quaternion
from math import *
import numpy as np

def execute(position, orientation, target, maxspeed):
    angle_to_target = atan2(target.y - position.y , target.x - position.x)
    distance_to_target = sqrt((position.x - target.x)**2 + (position.y - target.y)**2)
    yaw = np.radians(orientation.z)
    # yaw = yaw + np.radians(90) # test if this is correct
    # find the difference between orientation and angle to radius based on the yaw
    dyaw = angle_to_target-yaw
    # new_yaw = yaw + dyaw
    new_yaw = dyaw 
    print("yaw {} angle_to_target {} dyaw {} new_yaw {}".format( np.degrees(yaw), np.degrees(angle_to_target), np.degrees(dyaw), np.degrees(new_yaw)))

    # translate to linear x, y    
    twist = Twist()
    twist.linear.x = maxspeed*cos(new_yaw)
    twist.linear.y = maxspeed*sin(new_yaw)
    twist.angular.z = 0.0 # This could be used to change yaw.
    
    return twist, distance_to_target
    
if __name__ == '__main__':
    position = Point()
    orientation = Quaternion()
    target = Point()
    maxspeed = 0.01
    
    position.x = 0.0
    position.y = 0.0
    orientation.z = 0.0
    
    target.x = 1.0
    target.y = 1.0
    
    print(""" expect move with Twist:
    linear: x: 0.01, y: 0.01, z: 0.0""")
    print(execute(position, orientation, target, maxspeed))
    
    target.x = -1.0
    target.y = -1.0
    
    print("""\n expect move with Twist:
    linear: x: -0.01, y: -0.01, z: 0.0""")
    print(execute(position, orientation, target, maxspeed))
    
    target.x = 1.0
    target.y = 0.0
    
    print("""\n expect move with Twist:
    linear: x: 0.01, y: 0.0, z: 0.0""")
    print(execute(position, orientation, target, maxspeed))
    
    print("\n now with 90 degree angle")
    orientation.z = 360.0 + 100.0
    
    target.x = 1.0
    target.y = 1.0
    
    print("""\n expect move with Twist:
    linear: x: 0.01, y: 0.01, z: 0.0""")
    print(execute(position, orientation, target, maxspeed))
    
    target.x = -1.0
    target.y = -1.0
    
    print("""\n expect move with Twist:
    linear: x: -0.01, y: -0.01, z: 0.0""")
    print(execute(position, orientation, target, maxspeed))
    
    target.x = 1.0
    target.y = 0.0
    
    print("""\n expect move with Twist:
    linear: x: 0.01, y: 0.0, z: 0.0""")
    print(execute(position, orientation, target, maxspeed))
    
    print("""
        id: 0  
        x: 0.5794165760603266   
        y: 0.557869365624302  
        tx: 0.0         
        ty: -0.1        
        d: 0.8766502557122731   
        angle_to_target: -131.37189427923667  
        yaw: 179.99966406182594         
        new_yaw: -311.3715583410626     
        x: 0.006609394282809232      
        y: 0.007504392528004424  
        z: 0.0
    """)
    target.x = 0.0
    target.y = -1.0
    position.x = 0.57
    position.y = 0.55
    orientation.z = 180.0
    print(execute(position, orientation, target, maxspeed))
    
    
    print("""
        id: 1  
        x: -0.6186334857956306  
        y: -0.5337054495371448        
        tx: 0.0         
        ty: 0.2         
        d: 0.9597036398952318   
        angle_to_target: 49.863643239262856   
        yaw: 195.8600260424698  
        new_yaw: -145.99638280320696    
        x: -0.008290022679400624     
        y: -0.005592452411511726         
        z: 0.0""")
    target.x = 0.0
    target.y = -1.0
    position.x = 0.57
    position.y = 0.55
    orientation.z = 195.0
    print(execute(position, orientation, target, maxspeed))
    
    x= 0.4999268524970561   
    y= 0.6418793294547113         
    tx= 0.0         
    ty= 0.2         
    d= 0.6672212523945551   
    angle_to_target= -138.5269212393812   
    yaw= -0.0018320483879961103     
    new_yaw= -138.52508919099324    
    tw_x= -0.007492458029981428   
    tw_y= -0.0066229202523484165    
    tw_z= 0.0
    
    print(f"""
          x: {x}
          y: {y}
          tx: {tx}
          ty: {ty}
          d: {d}
          angle_to_target: {angle_to_target}
          yaw: {yaw}
          new_yaw: {new_yaw}
          tw_x: {tw_x}
          tw_y: {tw_y}
          tw_z: {tw_z}""")