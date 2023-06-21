import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Image, CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import math
import imutils


class FeducialFollower(Node):
    def __init__(self):
        super().__init__('feducial_follower')

        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        self.__camera_top_view_value = Image()
        self.create_subscription(Image, '/camera/top_view', self.__camera_top_view_callback, 1)
        
        self.__camera_top_view_info_value = CameraInfo()
        self.create_subscription(CameraInfo, '/camera/top_view/camera_info', self.__camera_top_view_info_callback, 1)
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        
        self.arucoDict = cv2.aruco.getPredefinedDictionary( cv2.aruco.DICT_5X5_250 )
        self.arucoParams = cv2.aruco.DetectorParameters()
        
        
        self.cameraMatrix = np.array([[16, 0, 1000 / 2],
                         [0, 16, 500 / 2],
                         [0, 0, 1]])
        self.distCoeffs = np.array([0,0,0,0,0])

        # Initialize the lists to store the coordinates
        self.x_coords = []
        self.y_coords = []
        self.connected_clients = []
        self.running = True
        self.inputX = 0
        self.inputY = 0
        self.IDinfoCurPos = {}
        self.IDinfoTargetPos = {}
        
        self.get_logger().info("FeducialFollower initialized")

    def __camera_top_view_info_callback(self, message):
        self.__camera_top_view_info_value = message
    def __camera_top_view_callback(self, message):
        self.__camera_top_view_value = message
        
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(message)
        
        current_frame = imutils.resize(current_frame, width=1000)
        
        # Display image
        # cv2.imshow("camera", current_frame)
        # cv2.waitKey(1)
        
        
        ### KLAAS CODE ###
        
        corners = []  # Define an empty list for corners
        (corners, ids, rejected) = cv2.aruco.detectMarkers(current_frame, self.arucoDict, parameters=self.arucoParams)

        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned
                # in top-left, top-right, bottom-right, and bottom-left
                # order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # calculate the center coordinates of the ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cX = math.floor(cX/10)
                cY = math.floor(cY/10)
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 1.0, cameraMatrix, distCoeffs)

                if len(self.x_coords) == 0 and len(self.y_coords) == 0:
                    # The list of coordinates is empty
                    # append the coordinates to the lists
                    self.x_coords.append(cX)
                    self.y_coords.append(cY)

                else:
                    rotation_matrix = np.zeros(shape=(3,3))
                    cv2.Rodrigues(rvecs,rotation_matrix)
                    # print(rotation_matrix)

                    # Assuming you already have the rotation matrix
                    rotation_matrix = np.zeros(shape=(3, 3))
                    cv2.Rodrigues(rvecs, rotation_matrix)

                    # Convert rotation matrix to Euler angles
                    sy = np.sqrt(rotation_matrix[0, 0] * rotation_matrix[0, 0] + rotation_matrix[1, 0] * rotation_matrix[1, 0])
                    singular = sy < 1e-6

                    if not singular:
                        x = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
                        y = np.arctan2(-rotation_matrix[2, 0], sy)
                        z = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
                    else:
                        x = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
                        y = np.arctan2(-rotation_matrix[2, 0], sy)
                        z = 0

                    # Convert angles from radians to degrees
                    x_deg = np.degrees(x)
                    y_deg = np.degrees(y)
                    z_deg = np.degrees(z)

                    # Print Euler angles in degrees
                    # print("Euler z-angle (in degrees):", z_deg)
                # Check if the current coordinates are different from the previous ones
                    if markerID in self.IDinfoCurPos:
                        # print("MARKERID: ",markerID)
                        if cX != self.IDinfoCurPos[markerID][0] or cY != self.IDinfoCurPos[markerID][1]:
                            # Update the coordinates in the dictionary
                            self.IDinfoCurPos[markerID] = [cX, cY]
                            print(self.IDinfoCurPos)
                            # print(IDinfo)
                            # Call the update_location() function
                            # update_location(markerID)
                    else:
                        # Add the new coordinates to the dictionary
                        self.IDinfoCurPos[markerID] = [cX, cY]
                        # print(self.IDinfoCurPos)
                        # Call the update_location() function
                        # update_location(markerID)

                        # if showPlot:
                        # # # redraw the plot
                        #     plt.cla()
                        #     plt.scatter(self.x_coords, self.y_coords, color='blue')  # Plot all dots in blue color
                        #     plt.scatter(self.x_coords[-1], self.y_coords[-1], color='red', marker='o', s=100)  # Plot the most recent dot in red color with a larger size
                        #     plt.gca().invert_yaxis()  # Invert the y-axis
                        #     plt.xlabel('X-coordinate')
                        #     plt.ylabel('Y-coordinate')
                        #     plt.title('ArUco Marker Coordinates')
                        #     plt.pause(0.001)

                        
                        # print the coordinates
                        # print("Marker ID:", markerID)
                        # print("Center coordinates (x, y):", cX, cY)

                        # convert each of the (x, y)-coordinate pairs to integers
                        topRight = (int(topRight[0]), int(topRight[1]))
                        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                        topLeft = (int(topLeft[0]), int(topLeft[1]))

                        # draw the bounding box of the ArUCo detection
                        cv2.line(current_frame, topLeft, topRight, (0, 255, 0), 2)
                        cv2.line(current_frame, topRight, bottomRight, (0, 255, 0), 2)
                        cv2.line(current_frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                        cv2.line(current_frame, bottomLeft, topLeft, (0, 255, 0), 2)

                        # compute and draw the center (x, y)-coordinates of the ArUco marker
                        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                        cv2.circle(current_frame, (cX, cY), 4, (0, 0, 255), -1)

                        # draw the ArUco marker ID on the current_frame
                        cv2.putText(current_frame, str(markerID), (topLeft[0], topLeft[1] - 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        # show the output frame
        cv2.imshow("frame", current_frame)
        key = cv2.waitKey(1) & 0xFF

    # do a bit of cleanup
    cv2.destroyAllWindows()




def main(args=None):
    rclpy.init(args=args)
    feducial_follower = FeducialFollower()
    rclpy.spin(feducial_follower)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    feducial_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()