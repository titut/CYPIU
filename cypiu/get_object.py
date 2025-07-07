"""
ROS node to generate random configurations for myCobot 280
"""

# ROS node import
import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# ROS Messages
from std_msgs.msg import Float32MultiArray

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from apriltag_msgs.msg import AprilTagDetectionArray
from apriltag_msgs.msg import AprilTagDetection

from cypiu.modules.fk import forward_kinematics
from cypiu.modules.ik import inverse_kinematics
from cypiu.modules.util import deg2rad, rad2deg

import numpy as np

class GetObject(Node):
    """
    Class to initialize and establish subscriber/publisher interaction
    """

    def __init__(self):
        super().__init__("get_object")

        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        self.current_angles = [-1, -1, -1, -1, -1, -1]

        self.publisher = self.create_publisher(Float32MultiArray, 'joint_angles', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.angle_subsciber = self.create_subscription(Float32MultiArray, 'current_angles', self.on_current_angles, 10)
        self.detection_subscriber = self.create_subscription(AprilTagDetectionArray, '/apriltag/detections', self.on_detections, qos_profile)

    def on_current_angles(self, msg):
        self.current_angles = msg.data

    def on_detections(self, msg: AprilTagDetectionArray):
        if self.current_angles[0] == -1:
            return
        try:
            t = self.tf_buffer.lookup_transform(
                "camera",
                "object",
                msg.header.stamp)
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
        desired_ee = t_sc[0:3, 3]
        if desired_ee[2] < 0.08:
            desired_ee[2] = 0.08
        self.get_logger().info(f"t_sb = {t_sb}")
        self.get_logger().info(f"t_bc = {t_bc}")
        self.get_logger().info(f"desired_ee = {desired_ee}")

        soln, status = inverse_kinematics([0, 0, 0, 0, 0, 0], desired_ee)
        
        msg = Float32MultiArray()
        msg.data = rad2deg(soln)
        self.publisher.publish(msg)


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
