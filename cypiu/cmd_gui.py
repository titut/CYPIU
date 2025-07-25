"""
ROS node to receive controller events and publish information
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from cypiu_interfaces.srv import Command
from std_srvs.srv import SetBool

import random
import time
import cypiu.modules.gpt as gpt


class CmdGui(Node):
    """
    Class to initialize and establish subscriber/publisher interaction
    """

    def __init__(self):
        super().__init__("cmd_gui")

        # Initialize publisher
        self.publisher = self.create_publisher(Float32MultiArray, "joint_angles", 10)

        # Initialize Apriltag Service Client
        self.cli = self.create_client(Command, "apriltag_service")
        self.claw_cli = self.create_client(SetBool, "claw_service")

        while True:
            user_input = input("Action: ")
            print("\n")
            if user_input == "random":
                self.random_location()
            elif user_input == "home":
                self.home()
            elif user_input == "look":
                self.look()
            elif user_input == "custom":
                self.custom()
            elif user_input == "claw open":
                self.claw(True)
            elif user_input == "claw close":
                self.claw(False)
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
        msg.data = [0.0, 45.0, -70.0, -30.0, 0.0, 0.0]
        self.publisher.publish(msg)

    def custom(self):
        joint_input = input("Joint no: ")
        amount_input = input("Amount: ")
        new_angles = [0.0, 45.0, -70.0, -30.0, 0.0, 0.0]
        new_angles[int(joint_input)] = float(amount_input)
        msg = Float32MultiArray()
        msg.data = new_angles
        self.publisher.publish(msg)

    def ask_gpt(self, sentence):
        self.claw(True)
        command = gpt.parse_command(sentence)
        self.get_logger().info(f"Parsed Command: {command}")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        req = Command.Request()
        req.action = command[0]
        req.object = command[1]
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f"Response: {response}")
        self.get_logger().info(f"{list(response.joint_angles)}")
        if response.success:
            msg = Float32MultiArray()
            msg.data = list(response.joint_angles)
            self.publisher.publish(msg)
        time.sleep(5)
        self.claw(False)
        time.sleep(3)
        self.look()

    def claw(self, bool):
        req = SetBool.Request()
        req.data = bool
        future = self.claw_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f"Response: {response}")


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
