import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


MAX_RANGE = 0.15


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        self.create_subscription(Range, 'range_right', self.__range_right_callback, 1)
        self.create_subscription(Range, 'range_left', self.__range_left_callback, 1)
        self.create_subscription(Range, 'range_front', self.__range_front_callback, 1)
        self.create_subscription(Range, 'range_back', self.__range_back_callback, 1)
        print("ObstacleAvoider initialized")

    def __range_left_callback(self, message):
        self.__range_left_value = message.range

    def __range_front_callback(self, message):
        self.__range_front_value = message.range
        
    def __range_back_callback(self, message):
        self.__range_back_value = message.range
        
    def __range_right_callback(self, message):
        self.__range_right_value = message.range

        # command_message = Twist()

        # command_message.linear.x = 0.1

        # if self.__left_sensor_value < 0.9 * MAX_RANGE or self.__right_sensor_value < 0.9 * MAX_RANGE:
        #     command_message.angular.z = -2.0

        # self.__publisher.publish(command_message)


def main(args=None):
    rclpy.init(args=args)
    avoider = ObstacleAvoider()
    rclpy.spin(avoider)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()