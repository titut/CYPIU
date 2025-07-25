"""
ROS node to use Apriltag libraries
"""

# ROS node import
import rclpy
from rclpy.node import Node

# ROS Messages
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool

from gpiozero import AngularServo


class ClawService(Node):
    """
    Class to initialize and establish subscriber/publisher interaction
    """

    def __init__(self):
        super().__init__("claw_service")

        # Inititalize servo connection GPIO Pin #18
        self.servo = AngularServo(18, min_pulse_width=0.0006, max_pulse_width=0.0023)

        self.srv = self.create_service(SetBool, "claw_service", self.claw)

    def claw(self, request, response):
        """
        True: Open Claw
        False: Close Claw
        """
        if request.data:
            self.servo.angle = 90
            response.success = True
            response.message = "Claw opened!"
        else:
            self.servo.angle = -90
            response.success = True
            response.message = "Claw closed!"

        return response


def main(args=None):
    rclpy.init(args=args)

    claw_service = ClawService()

    rclpy.spin(claw_service)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    claw_service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
