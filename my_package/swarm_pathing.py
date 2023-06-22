import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Image, CameraInfo
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
import math


class SwarmPathing(Node):
    def __init__(self):
        super().__init__("swarm_pathing")

        self.__selected_swarm_goal = self.__empty_func
        self.__set_swarm_goal_data = String()
        self.create_subscription(
            String, "/set_swarm_goal", self.__set_swarm_goal_callback, 1
        )

        # create function dictionary
        self.__swarm_goal_dict = {
            "straightline": self.__straightline,
            "circle": self.__circle,
        }

        # subscribe to new camera data
        self.__camera_position_subscriber_dict = dict()

        # variable to store latest camera data
        self.__camera_position_data = dict()

        # Create a timer that fires every second to check for new topics
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.check_for_new_topics)

        self.get_logger().info("SwarmPathing initialized")

    def check_for_new_topics(self):
        topics = self.get_topic_names_and_types()
        for topic in topics:
            # check if already known
            if topic[0] in self.__camera_position_subscriber_dict.keys():
                continue
            # check if topic starts with /aruco/feducial
            if topic[0].startswith("/aruco/feducial"):
                t = topic[0]
                self.__camera_position_subscriber_dict[
                    topic[0]
                ] = self.create_subscription(
                    Point,
                    topic[0],
                    lambda msg: self.__camera_position_callback(msg, t),
                    1,
                )
                self.get_logger().info("Topic: " + topic[0] + " added")

        # check if anything went offline
        for topic in self.__camera_position_subscriber_dict.keys():
            if topic not in [i[0] for i in topics]:
                self.__camera_position_subscriber_dict[topic].destroy()
                self.__camera_position_subscriber_dict.pop(topic)
                self.get_logger().info("Topic " + topic + " went offline")

    def __camera_position_callback(self, message, topic):
        # check which drone sent the message
        self.__camera_position_data[topic] = message

    def __set_swarm_goal_callback(self, message):
        self.__set_swarm_goal_data = message

        if message.data in self.__swarm_goal_dict.keys():
            self.__selected_swarm_goal = self.__swarm_goal_dict[message.data]

    def __empty_func(self):
        pass

    def __straightline(self, markerID, cX, cY):
        # set target pos to:
        arr = [
            [10, 25],
            [20, 25],
            [30, 25],
            [40, 25],
            [50, 25],
            [60, 25],
            [70, 25],
            [80, 25],
            [90, 25],
            [100, 25],
        ]
        # check if position is reached +- 1 coord

        # set next target pos to:
        return arr

    def __circle(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    swarm_pathing = SwarmPathing()
    rclpy.spin(swarm_pathing)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cv2.destroyAllWindows()
    swarm_pathing.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
