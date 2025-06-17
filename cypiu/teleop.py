"""
ROS node to receive controller events and publish information
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

from pydualsense import pydualsense
from pymycobot import MyCobot280
import random

class Teleop(Node):
    """
    Class to initialize and establish subscriber/publisher interaction
    """

    def __init__(self):
        super().__init__("teleop")

        # Initialize controller
        self.get_logger().info("Connecting to Robot...")
        self.mc = MyCobot280("/dev/ttyAMA0", 1000000) # !IMPORTANT ONLY USE READ FUNCTIONS ONLY
        self.get_logger().info("Connected!")

        # Initialize dualsense controller
        self.get_logger().info("Connecting to Controller")
        self.ds = pydualsense()
        self.ds.init()
        self.ds.circle_pressed += self.read_joint_angles
        self.ds.cross_pressed += self.go_home
        self.ds.square_pressed += self.random_location
        self.get_logger().info("Connected")

        # Initialize publisher
        self.publisher = self.create_publisher(Float32MultiArray, 'joint_angles', 10)

    def read_joint_angles(self, state):
        if state:
            self.get_logger().info(f"Current Robot Orientation: {self.mc.get_angles()}")

    def go_home(self, state):
        if state:
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.publisher.publish(msg)
            self.get_logger().info(f"Sent Command to go home!")

    def random_location(self, state):
        if state:
            random_joint_angles = []
            for i in range(5):
                random_joint_angles.append(float(random.randrange(0, 90)))
            random_joint_angles.append(0.0)
            self.get_logger().info(f"Published random angles: {random_joint_angles}")
            msg = Float32MultiArray()
            msg.data = random_joint_angles
            self.publisher.publish(msg)

    def on_dpad_up(self, state):
        if state:
            cur_coords = self.mc.get_coords()
            cur_coords[0] = cur_coords[0] + 10
            msg = Float32MultiArray()
            msg.data = self.mc.solve_inv_kinematics(cur_coords, self.mc.get_angles())
            self.get_logger().info(f"Published new angles: {msg.data}")
            self.publisher.publish(msg)

    def on_dpad_down(self, state):
        self.get_logger().info(f"Current Robot Orientation: {self.mc.get_angles()}")

    def on_dpad_left(self, state):
        self.get_logger().info(f"Current Robot Orientation: {self.mc.get_angles()}")

    def on_dpad_right(self, state):
        self.get_logger().info(f"Current Robot Orientation: {self.mc.get_angles()}")

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
