#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from math import sin, cos, pi
from squaternion import Quaternion
from crazyflie_swarm_interfaces.msg import MoveDrone

class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_msg_TEST')
        self.publisher_ = self.create_publisher(MoveDrone, '/supervisor/move_drone', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.radius = 0.5
        self.increment = 0.5

    def timer_callback(self):
        msg = MoveDrone()
        msg.marker = 1
        msg.header.stamp = self.get_clock().now().to_msg()    

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Path')


def main(args=None):
    rclpy.init(args=args)

    viz_publisher = PathPublisher()

    rclpy.spin(viz_publisher)

    viz_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()