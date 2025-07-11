"""
ROS node to receive controller events and publish information
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool

import random
import cypiu.modules.gpt as gpt

class CmdGui(Node):
    """
    Class to initialize and establish subscriber/publisher interaction
    """

    def __init__(self):
        super().__init__("cmd_gui")

        # Initialize publisher
        self.publisher = self.create_publisher(Float32MultiArray, 'joint_angles', 10)

        # Initialize Apriltag Service Client
        self.cli = self.create_client(SetBool, 'apriltag_service')

        while True:
            user_input = input("Action: ")
            print("\n")
            if user_input == "random":
                self.random_location()
            if user_input == "home":
                self.home()
            if user_input == "look":
                self.look()
            else:
                self.ask_gpt(user_input)

    def random_location(self):
        random_joint_angles = []
        for i in range(5):
            random_joint_angles.append(float(random.randrange(-40, 40)))
        random_joint_angles.append(0.0)
        self.get_logger().info(f"Published random angles: {random_joint_angles}")
        msg = Float32MultiArray()
        msg.data = random_joint_angles
        self.publisher.publish(msg)

    def home(self):
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.publisher.publish(msg)

    def look(self):
        msg = Float32MultiArray()
        msg.data = [0.0, 45.0, -70.0, -40.0, 0.0, 0.0]
        self.publisher.publish(msg)

    def ask_gpt(self, sentence):
        command = gpt.parse_command(sentence)
        self.get_logger().info(f"Parsed Command: {command}")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        req = SetBool.Request()
        req.data = True
        response = self.cli.call_async(req).result()
        print(response)
        

def main(args=None):
    rclpy.init(args=args)

    cmd_gui = CmdGui()

    rclpy.spin(cmd_gui)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cmd_gui.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
