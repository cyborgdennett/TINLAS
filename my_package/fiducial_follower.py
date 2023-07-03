import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Image, CameraInfo
from geometry_msgs.msg import Twist
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection, Point
from std_msgs.msg import String
from aruco_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
import math


# import ParameterEventHandler # not yet supported on rclpy


class FiducialFollower(Node):

    def __init__(self):
        super().__init__("fiducial_follower")

        # declare params
        # self.declare_parameters(
        #     namespace="",
        #     parameters=[
        #         ("camera_topic", "/camera/top_view"),
        #         ("apriltag_array_topic", "/fiducial/apriltag_array"),
        #         ("apriltag_overlay_publish_enable", False),
        #         ("apriltag_overlay_topic", "/fiducial/overlay"),                
        #     ]
        # )    
        # # for performance it would be better to turn overlay publish off, and make an overlay using the apriltag overlay plugin with rqt
        # camera_topic, apriltag_array_topic, self.apriltag_overlay_publish_enable, apriltag_overlay_topic = self.get_parameters([
        #         "camera_topic",
        #         "apriltag_array_topic",
        #         "apriltag_overlay_publish_enable",
        #         "apriltag_overlay_topic",
        # ])
        
        camera_topic, apriltag_array_topic, self.apriltag_overlay_publish_enable, apriltag_overlay_topic, arcuo_array_topic = [
                "/camera/top_view",
                "/fiducial/apriltag_array",
                False,
                "/fiducial/overlay",    
                "/fiducial/aruco_array",
        ]
        
        self.marker_length = 0.024 # in meters. (the markerLength is counting without the white border)


        self.create_subscription(
            Image, camera_topic, self.__camera_input_callback, 1
        )

        self.create_subscription(
            CameraInfo,
            camera_topic + "/camera_info",
            self.__camera_input_info_callback,
            1,
        )
        
        # publish the image with the overlay
        if self.apriltag_overlay_publish_enable:
            self.__apriltag_overlay_publisher = self.create_publisher(Image, apriltag_overlay_topic, 1)
        
        self.__apriltag_array_publisher = self.create_publisher(AprilTagDetectionArray, apriltag_array_topic, 1)
        
        self.__aruco_array_publisher = self.create_publisher(MarkerArray, arcuo_array_topic, 1)
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        self.arucoDict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_ARUCO_ORIGINAL
        )
        self.arucoParams = cv2.aruco.DetectorParameters()

        # TODO get this data ffrom the camera info topic
        self.cameraMatrix = np.array([[16, 0, 1000 / 2], [0, 16, 500 / 2], [0, 0, 1]])
        self.distCoeffs = np.array([0, 0, 0, 0, 0])

        self.get_logger().info("FiducialFollower initialized")

    def __camera_input_info_callback(self, message):
        #TODO? get camera info
        pass
    
    def __camera_input_callback(self, message):
        # make msg to publish
        
        apriltag_msg = AprilTagDetectionArray()
        apriltag_msg.header.stamp = self.get_clock().now().to_msg()
        apriltag_msg.header.frame_id = "camera"
        apriltag_msg.detections = []
        
        aruco_msg = MarkerArray()
        aruco_msg.header.stamp = self.get_clock().now().to_msg()
        aruco_msg.header.frame_id = "camera"
        aruco_msg.markers = []
        
        # Convert ROS Image message to OpenCV image

        current_frame = self.br.imgmsg_to_cv2(message, "bgr8")

        ### KLAAS CODE ###

        corners = []  # Define an empty list for corners
        corners, ids, _ = cv2.aruco.detectMarkers(
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
                (topLeft, topRight, bottomRight, bottomLeft) = markerCorner.reshape((4, 2))

                # calculate the center coordinates of the ArUco marker
                cX = (topLeft[0] + bottomRight[0]) / 2.0
                cY = (topLeft[1] + bottomRight[1]) / 2.0
                # cX = math.floor(cX / 10)
                # cY = math.floor(cY / 10)
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    markerCorner, self.marker_length, self.cameraMatrix, self.distCoeffs
                )

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
                
                detection = AprilTagDetection()
                detection.id = int(markerID)
                # detection.centre = Point()
                detection.centre.x = cX
                detection.centre.y = cY
                # detection.corners = [Point(topLeft), Point(topRight), Point(bottomRight), Point(bottomLeft)]
                # todo add the other things to the detection
                
                apriltag_msg.detections.append(detection)
                
                aruco_marker = Marker()
                aruco_marker.id = int(markerID)
                aruco_marker.pose.pose.position.x = tvecs[0][0][0]
                aruco_marker.pose.pose.position.y = tvecs[0][0][1]
                aruco_marker.pose.pose.position.z = tvecs[0][0][2]
                aruco_marker.pose.pose.orientation.x = x_deg
                aruco_marker.pose.pose.orientation.y = y_deg
                aruco_marker.pose.pose.orientation.z = z_deg
                aruco_marker.pose.pose.orientation.w = 1.0
                
                aruco_msg.markers.append(aruco_marker)
                

                if self.apriltag_overlay_publish_enable:
                    self.__draw_fiducial_bounding_box(
                        markerID,
                        topRight,
                        bottomRight,
                        bottomLeft,
                        topLeft,
                        current_frame,
                    )    
        
        
        if self.apriltag_overlay_publish_enable:
            img_msg = self.br.cv2_to_imgmsg(current_frame, "bgr8")
            self.__publish_fiducial_view(img_msg)
        
        self.__apriltag_array_publisher.publish(apriltag_msg)
        self.__aruco_array_publisher.publish(aruco_msg)
        # show the output frame
        # if self.fiducial_overlay_imshow_enable:
        #     cv2.imshow("frame", current_frame)
        #     key = cv2.waitKey(1) & 0xFF

    def __cam_out_publish(self, markerID, cX, cY):
        if markerID not in self.__fiducial_publisher_dict:
            self.__fiducial_publisher_dict[markerID] = self.create_publisher(
                Point, f"/aruco/fiducial{markerID}", 10
            )
        # TODO: transform the coordinates to the webots coordinate system
        message = Point()
        message.x = float(cX)
        message.y = float(cY)
        self.__fiducial_publisher_dict[markerID].publish(message)

    def __draw_fiducial_bounding_box(
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
    fiducial_follower = FiducialFollower()
    rclpy.spin(fiducial_follower)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cv2.destroyAllWindows()
    fiducial_follower.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
