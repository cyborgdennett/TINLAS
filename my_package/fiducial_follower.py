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
from crazyflie_swarm_interfaces.msg import MoveDrone
import random
import time
import tempfile
import copy
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
            Image, camera_topic, self.camera_image_callback, 1
        )

        self.create_subscription(
            CameraInfo,
            camera_topic + "/camera_info",
            self.__camera_input_info_callback,
            1,
        )
        self.camera_info = None
        self.current_frame = None
        
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
        if self.init_calibrate() != "success":
            self.calibrate_timer = self.create_timer(0.2, self.calibrate)
        
    
    def init_calibrate(self):
        path = tempfile.gettempdir() + "/calibration.yaml"
        cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

        if cv_file.getNode("K") is not None:
            # no need for calibration, use the values from the file
            self.cameraMatrix = cv_file.getNode("K").mat()
            self.distCoeffs = cv_file.getNode("D").mat()
            return "success"
        self.get_logger().info("Calibrating camera")
        self.positions = [ # camera is from 2.2 z and 0,0 xy
            (0,0,0.15),
            (0,0,0.3),
            (0,0,0.45),
            (0,0,0.6),
            (0,0,0.75),
            (0,0,0.9),
            (0,0,1.05),
            (0,0,1.2),
            (0,0,1.35),
            (0,0,1.5),
            
            (0,0.15,0.15),
            (0,0.15,0.3),
            (0,0.15,0.45),
            (0,0.15,0.6),
            (0,0.15,0.75),
            (0,0.15,0.9),
            (0,0.15,1.05),
            (0,0.15,1.2),
            (0,0.15,1.35),
            (0,0.15,1.5),
            
            (0,0.3,0.15),
            (0,0.3,0.3),
            (0,0.3,0.45),
            (0,0.3,0.6),
            (0,0.3,0.75),
            (0,0.3,0.9),
            (0,0.3,1.05),
            (0,0.3,1.2),
            (0,0.3,1.35),
            (0,0.3,1.5),
            
            (0,3.3,0.15),
            (0,3.3,0.3),
            (0,3.3,0.45),
            (0,3.3,0.6),
            (0,3.3,0.75),
            (0,3.3,0.9),
            (0,3.3,1.05),
            (0,3.3,1.2),
            (0,3.3,1.35),
            (0,3.3,1.5),
            
            (0.15,0,0.15),
            (0.15,0,0.3),
            (0.15,0,0.45),
            (0.15,0,0.6),
            (0.15,0,0.75),
            (0.15,0,0.9),
            (0.15,0,1.05),
            (0.15,0,1.2),
            (0.15,0,1.35),
            (0.15,0,1.5),
        ]
        self.orientations = [
            (0, 0, 1, 0),
            (-0.3244409918936797, -0.20083699498198732, -0.9243389769049288, 1.66708),
            (-0.20450492836377665, -0.0819909712793057, -0.9754256583172305, 1.62987),
            (0.31579285904816884, -0.044616580085716055, 0.9477785769659696, -1.6982953071795865),
            (0.05436968156163775, 0.16310994468460804, -0.9851086659206029, 1.65155),
            (0.048792200560885866, -0.08869560101959141, -0.9948630114363484, 1.62104),
        ]
        # Arrays to store object points and image points from all the images.
        self.objpoints = []  # 3d point in real world space
        self.imgpoints = []  # 2d points in image plane.
        # termination self.criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        self.supervisor_remover = self.create_publisher(String, "/remove_urdf_robot", 1) # removes by name
        self.supervisor_spawner = self.create_publisher(MoveDrone, "/supervisor/chessboard", 1) # just spawns a new one
        self.calibrate_i = 0
        
    """ https://aliyasineser.medium.com/opencv-camera-calibration-e9a48bdd1844 """
    def calibrate(self):
        
        if self.camera_info is None or self.current_frame is None:
            return -1

        md = MoveDrone()
        md.pose.position.x = 0.0
        md.pose.position.y = 0.0
        md.pose.position.z = 0.15
        md.pose.orientation.x = 0.0
        md.pose.orientation.y = 0.0
        md.pose.orientation.z = 1.0
        md.pose.orientation.w = 0.0
        
        # chessboard size (only count inner corner)
        height = 8 - 1
        width = 5 - 1
        # square size in meter
        square_size = 0.0625
        
        objp = np.zeros((height*width, 3), np.float32)
        objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

        objp = objp * square_size
        
        img = self.current_frame
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)
        
        cv2.imshow("a", img)
        cv2.waitKey(1)
        pos = self.positions[self.calibrate_i]

        # If found, add object points, image points (after refining them)
        if ret:
            self.objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
            self.imgpoints.append(corners2)
            
            img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)
            cv2.imshow("a", img)
            cv2.waitKey(1)

            self.get_logger().info("Found chessboard at position: " + str(pos))                
        else:
            self.get_logger().info("Found nothing position: " + str(pos))                
            
        # move the board
        pos = self.positions[self.calibrate_i]
        self.calibrate_i += 1
        # get random orientation
        orientation = self.orientations[random.randint(0, len(self.orientations) - 1)]
        # fill message
        md.pose.position.x = float(pos[0])
        md.pose.position.y = float(pos[1])
        md.pose.position.z = float(pos[2])
        md.pose.orientation.x = float(orientation[0])
        md.pose.orientation.y = float(orientation[1])
        md.pose.orientation.z = float(orientation[2])
        md.pose.orientation.w = float(orientation[3])
        # publish message
        self.supervisor_spawner.publish(md)
            
        if self.calibrate_i < len(self.positions) - 1:
            return
                
        # calibrate camera
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, gray.shape[::-1], None, None)
        
        # make file
        temp = tempfile.NamedTemporaryFile(delete=False)
        temp.name = tempfile.gettempdir() + "/calibration.yaml"
        temp.close()
        
        # save calibration
        cv_file = cv2.FileStorage(temp.name, cv2.FILE_STORAGE_WRITE)
        cv_file.write("K", mtx)
        cv_file.write("D", dist)
        # note you *release* you don't close() a FileStorage object
        cv_file.release()
        
        # store to local variables
        self.cameraMatrix = mtx
        self.distCoeffs = dist
            
        # remove chessboard from world
        msg = String()
        msg.data = "chessboard"
        self.supervisor_remover.publish(msg)
                
        self.destroy_publisher(self.supervisor_remover)
        self.destroy_publisher(self.supervisor_spawner)
        
        self.get_logger().info("Calibration done and saved to %s" % temp.name)
        
        self.calibrate_timer.destroy()
        
        

    def __camera_input_info_callback(self, message):
        self.camera_info = message
        # self.get_logger().info("Camera info received %s" % message)
        if self.cameraMatrix is None:
            self.cameraMatrix = np.array(message.k).reshape((3, 3))
            self.distCoeffs = np.array(message.d)
        
    
    def camera_image_callback(self, message):
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

        self.current_frame = self.br.imgmsg_to_cv2(message, "bgr8")

        ### KLAAS CODE ###

        corners = []  # Define an empty list for corners
        corners, ids, _ = cv2.aruco.detectMarkers(
            self.current_frame, self.arucoDict, parameters=self.arucoParams
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
                
                """
                Camera info received sensor_msgs.msg.CameraInfo(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1688499984, nanosec=917116809), frame_id='camera_top_link'), height=1000, width=1000, distortion_model='plumb_bob', d=[0.0, 0.0, 0.0, 0.0, 0.0], k=array([1.0759939e+03, 0.0000000e+00, 5.0000000e+02, 0.0000000e+00,
                [fiducial_follower-4]        1.0759939e+03, 5.0000000e+02, 0.0000000e+00, 0.0000000e+00,
                [fiducial_follower-4]        1.0000000e+00]), r=array([1., 0., 0., 0., 1., 0., 0., 0., 1.]), p=array([1.0759939e+03, 0.0000000e+00, 5.0000000e+02, 0.0000000e+00,
                [fiducial_follower-4]        0.0000000e+00, 1.0759939e+03, 5.0000000e+02, 0.0000000e+00,
                [fiducial_follower-4]        0.0000000e+00, 0.0000000e+00, 1.0000000e+00, 0.0000000e+00]), binning_x=0, binning_y=0, roi=sensor_msgs.msg.RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))
                """
                
                # rvecs and tvecs are both Vector3 but show as [[[ ]]]
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
                
                # only Z matters
                # self.get_logger ().info("Marker ID: " + str(markerID) + " rotation x: " + str(x_deg) + " y: " + str(y_deg) + " z: " + str(z_deg))
                
                detection = AprilTagDetection()
                detection.id = int(markerID)
                # detection.centre = Point()
                detection.centre.x = cX
                detection.centre.y = cY
                detection.corners[0].x = float(topLeft[0])
                detection.corners[0].y = float(topLeft[1])
                detection.corners[1].x = float(topRight[0])
                detection.corners[1].y = float(topRight[1])
                detection.corners[2].x = float(bottomRight[0])
                detection.corners[2].y = float(bottomRight[1])
                detection.corners[3].x = float(bottomLeft[0])
                detection.corners[3].y = float(bottomLeft[1])
                
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
                        self.current_frame,
                    )    
        
        
        if self.apriltag_overlay_publish_enable:
            img_msg = self.br.cv2_to_imgmsg(self.current_frame, "bgr8")
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
