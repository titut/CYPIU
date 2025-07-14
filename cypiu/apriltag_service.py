"""
ROS node to use Apriltag libraries
"""

# ROS node import
import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# ROS Messages
from std_msgs.msg import Float32MultiArray
from cypiu_interfaces.srv import Command

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from apriltag_msgs.msg import AprilTagDetectionArray

from cypiu.modules.fk import forward_kinematics
from cypiu.modules.ik import inverse_kinematics
from cypiu.modules.util import deg2rad, rad2deg

import numpy as np

class ApriltagService(Node):
    """
    Class to initialize and establish subscriber/publisher interaction
    """

    def __init__(self):
        super().__init__("apriltag_service")

        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        self.current_angles = [-1, -1, -1, -1, -1, -1]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.angle_subsciber = self.create_subscription(Float32MultiArray, 'current_angles', self.on_current_angles, 10)
        self.srv = self.create_service(Command, 'apriltag_service', self.find_object)

    def on_current_angles(self, msg):
        self.current_angles = msg.data

    def find_object(self, request, response):
        self.get_logger().info(f"Request: {request}")
        if self.current_angles[0] == -1:
            response.success = False
            response.message = "Current angles not initialized."
            response.joint_angles = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
            return response
        # Find Transform
        try:
            t = self.tf_buffer.lookup_transform(
                "camera",
                request.object,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform camera to object: {ex}')
            response.success = False
            response.message = "Unable to find transform."
            response.joint_angles = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
            return response
    
        # Determine Apriltag pose with regards to World frame
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

        # Inverse Kinematics
        soln, status = inverse_kinematics([0, 0, 0, 0, 0, 0], desired_ee)
        soln[5] = 0

        response.success = True
        response.message = f"Valid solution reached: {str(rad2deg(soln))}"
        response.joint_angles = rad2deg(soln)

        self.get_logger().info(f"Response = {response}")

        return response


def main(args=None):
    rclpy.init(args=args)

    apriltag_service = ApriltagService()

    rclpy.spin(apriltag_service)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    apriltag_service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
