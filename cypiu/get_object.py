"""
ROS node to generate random configurations for myCobot 280
"""

# ROS node import
import rclpy
from rclpy.node import Node

# ROS Messages
from std_msgs.msg import Float32MultiArray

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from cypiu.modules.fk import forward_kinematics
from cypiu.modules.util import deg2rad

import numpy as np

class GetObject(Node):
    """
    Class to initialize and establish subscriber/publisher interaction
    """

    def __init__(self):
        super().__init__("get_object")
        self.current_angles = [0,0,0,0,0,0]

        self.publisher = self.create_publisher(Float32MultiArray, 'joint_angles', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subsciber = self.create_subscription(Float32MultiArray, 'current_angles', self.on_current_angles, 10)

        self.timer = self.create_timer(0.1, self.on_timer)

    def on_current_angles(self, msg):
        self.current_angles = msg.data

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "object",
                "camera",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform camera to object: {ex}')
            return

        t_bc = np.array([
            [1, 0, 0, t.transform.translation.x],
            [0, 1, 0, t.transform.translation.y],
            [0, 0, 1, t.transform.translation.z],
            [0, 0, 0, 1],
        ])

        cur_angles = deg2rad(self.current_angles)
        cur_coords, t_sb = forward_kinematics(cur_angles)

        t_sc = t_sb @ t_bc
        self.get_logger().info(f"T_sb = {t_sb}")
        self.get_logger().info(f"T_bc = {t_bc}")
        self.get_logger().info(f"T_sc = {t_sc}")

def main(args=None):
    rclpy.init(args=args)

    get_object = GetObject()

    rclpy.spin(get_object)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    get_object.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
