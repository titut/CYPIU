"""
ROS node to move myCobot 280 arm
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class MoveArm(Node):
    """
    Class to initialize and establish subscriber/publisher interaction
    """

    def __init__(self):
        super().__init__("Move Arm")
        self.subscription = self.create_subscription(
            Float32MultiArray, "joint_angles", self.listener_callback, 10
        )

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    move_arm = MoveArm()

    rclpy.spin(move_arm)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move_arm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
