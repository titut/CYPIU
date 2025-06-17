"""
ROS node to generate random configurations for myCobot 280
"""

# ROS node import
import rclpy
from rclpy.node import Node

# ROS Messages
from std_msgs.msg import Float32MultiArray

# Non-ROS import
import random

class RandomLoc(Node):
    """
    Class to initialize and establish subscriber/publisher interaction
    """

    def __init__(self):
        super().__init__("random_loc")
        timer_period = 0.5  # seconds
        self.publisher = self.create_publisher(Float32MultiArray, 'joint_angles', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        random_joint_angles = []
        for i in range(5):
            random_joint_angles.append(float(random.randrange(0, 90)))
        random_joint_angles.append(0.0)
        self.get_logger().info(f"Published random angles: {random_joint_angles}")
        msg = Float32MultiArray()
        msg.data = random_joint_angles
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    random_loc = RandomLoc()

    rclpy.spin(random_loc)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    random_loc.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
