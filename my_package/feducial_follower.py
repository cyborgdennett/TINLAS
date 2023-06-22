import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Image, CameraInfo
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
import math


class FeducialFollower(Node):
    def __init__(self):
        super().__init__("feducial_follower")

        # TODO: make a startup parameter for this
        self.show_camera_output = True

        # TODO: make a paramater to change the camera topic

        # publish the x,y coordinates of the drone
        self.__cam_out_publisher_dict = dict()

        self.__camera_top_view_value = Image()
        self.create_subscription(
            Image, "/camera/top_view", self.__camera_top_view_callback, 1
        )

        self.__camera_top_view_info_value = CameraInfo()
        self.create_subscription(
            CameraInfo,
            "/camera/top_view/camera_info",
            self.__camera_top_view_info_callback,
            1,
        )

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        self.arucoDict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_ARUCO_ORIGINAL
        )
        self.arucoParams = cv2.aruco.DetectorParameters()

        # TODO get this data ffrom the camera info topic
        self.cameraMatrix = np.array([[16, 0, 1000 / 2], [0, 16, 500 / 2], [0, 0, 1]])
        self.distCoeffs = np.array([0, 0, 0, 0, 0])

        # Initialize the lists to store the coordinates
        self.x_coords = []
        self.y_coords = []
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

        current_frame = self.br.imgmsg_to_cv2(message, "bgr8")

        ### KLAAS CODE ###

        corners = []  # Define an empty list for corners
        (corners, ids, rejected) = cv2.aruco.detectMarkers(
            current_frame, self.arucoDict, parameters=self.arucoParams
        )

        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for markerCorner, markerID in zip(corners, ids):
                # extract the marker corners (which are always returned
                # in top-left, top-right, bottom-right, and bottom-left
                # order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # calculate the center coordinates of the ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cX = math.floor(cX / 10)
                cY = math.floor(cY / 10)
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    markerCorner, 1.0, self.cameraMatrix, self.distCoeffs
                )

                if len(self.x_coords) == 0 and len(self.y_coords) == 0:
                    # The list of coordinates is empty
                    # append the coordinates to the lists
                    self.x_coords.append(cX)
                    self.y_coords.append(cY)

                else:
                    rotation_matrix = np.zeros(shape=(3, 3))
                    cv2.Rodrigues(rvecs, rotation_matrix)
                    # print(rotation_matrix)

                    # Assuming you already have the rotation matrix
                    rotation_matrix = np.zeros(shape=(3, 3))
                    cv2.Rodrigues(rvecs, rotation_matrix)

                    # Convert rotation matrix to Euler angles
                    sy = np.sqrt(
                        rotation_matrix[0, 0] * rotation_matrix[0, 0]
                        + rotation_matrix[1, 0] * rotation_matrix[1, 0]
                    )
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
                    self.__draw_feducial_bounding_box(
                        markerID,
                        topRight,
                        bottomRight,
                        bottomLeft,
                        topLeft,
                        current_frame,
                    )
                    # print("Euler z-angle (in degrees):", z_deg)
                    # Check if the current coordinates are different from the previous ones
                    if markerID in self.IDinfoCurPos:
                        if (
                            cX != self.IDinfoCurPos[markerID][0]
                            or cY != self.IDinfoCurPos[markerID][1]
                        ):
                            # Update the coordinates in the dictionary
                            self.IDinfoCurPos[markerID] = [cX, cY]
                            # self.get_logger().info(f"marker new pos: {markerID}: [{cX}, {cY}]")

                            # publish the new coordinates
                            self.__cam_out_publish(markerID, cX, cY)

                    else:
                        # Add the new coordinates to the dictionary
                        self.IDinfoCurPos[markerID] = [cX, cY]
                        # create new publisher for the said marker
                        self.__cam_out_publish(markerID, cX, cY)

        # show the output frame
        cv2.imshow("frame", current_frame)
        key = cv2.waitKey(1) & 0xFF

    def __cam_out_publish(self, markerID, cX, cY):
        if markerID not in self.__cam_out_publisher_dict:
            self.__cam_out_publisher_dict[markerID] = self.create_publisher(
                Point, f"/aruco/feducial{markerID}", 10
            )
        # TODO: transform the coordinates to the webots coordinate system
        message = Point()
        message.x = float(cX)
        message.y = float(cY)
        self.__cam_out_publisher_dict[markerID].publish(message)

    def __draw_feducial_bounding_box(
        self, markerID, topRight, bottomRight, bottomLeft, topLeft, current_frame
    ):
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
        cv2.putText(
            current_frame,
            str(markerID),
            (topLeft[0], topLeft[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )


def main(args=None):
    rclpy.init(args=args)
    feducial_follower = FeducialFollower()
    rclpy.spin(feducial_follower)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cv2.destroyAllWindows()
    feducial_follower.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
