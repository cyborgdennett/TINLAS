import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Image, CameraInfo
from geometry_msgs.msg import Twist


class FeducialFollower(Node):
    def __init__(self):
        super().__init__('feducial_follower')

        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        self.__camera_top_view_value = Image()
        self.create_subscription(Image, '/camera/top_view', self.__camera_top_view_callback, 1)
        self.__camera_top_view_info_value = CameraInfo()
        self.create_subscription(CameraInfo, '/camera/top_view/camera_info', self.__camera_top_view_info_callback, 1)
        
        print("FeducialFollower initialized")

    def __camera_top_view_info_callback(self, message):
        self.__camera_top_view_info_value = message
    def __camera_top_view_callback(self, message):
        self.__camera_top_view_value = message
        
        print(message.encoding)

        # command_message.linear.x = 0.1

        # if self.__left_sensor_value < 0.9 * MAX_RANGE or self.__right_sensor_value < 0.9 * MAX_RANGE:
        #     command_message.angular.z = -2.0

        # self.__publisher.publish(command_message)


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