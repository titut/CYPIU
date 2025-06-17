"""
ROS node to receive controller events and publish information
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

from pydualsense import pydualsense

class Teleop(Node):
    """
    Class to initialize and establish subscriber/publisher interaction
    """

    def __init__(self):
        super().__init__("teleop")
        # Initialize controller
        self.ds = pydualsense()
        self.ds.init()
        self.ds.circle_pressed += self.on_circle_pressed

    def on_circle_pressed(self, state):
        self.get_logger().info(f"Circle button {'pressed' if state else 'released'}")

def main(args=None):
    rclpy.init(args=args)

    teleop = Teleop()

    rclpy.spin(teleop)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    teleop.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
