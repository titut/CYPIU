"""
ROS node to move myCobot 280 arm
"""

import rclpy
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np

class DrawCorners(Node):
    """
    Class to initialize and establish subscriber/publisher interaction
    """

    def __init__(self):
        super().__init__("draw_corners")
        self.detection = None
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(Image, "/camera1/image_raw", self.image_callback, 10)
        self.detection_subscriber = self.create_subscription(AprilTagDetectionArray, "/apriltag/detections", self.detection_callback, 10)
        self.publisher = self.create_publisher(Image, "/corners", 10)

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg)
        if self.detection is not None:
            pts = np.array(self.detection, np.int32)
            pts = pts.reshape((-1,1,2))
        else:
            pts = np.array([[10,5],[20,30],[70,20],[50,10]], np.int32)
            pts = pts.reshape((-1,1,2))
        frame = cv2.polylines(frame,[pts],True,(0,255,255))
        
        imgmsg = self.bridge.cv2_to_imgmsg(frame)
        self.publisher.publish(imgmsg)
        

    def detection_callback(self, msg: AprilTagDetectionArray):
        detections = msg.detections
        for i in detections:
            self.detection = np.array([
                [i.corners[0][0], i.corners[0][1]],
                [i.corners[1][0], i.corners[1][1]],
                [i.corners[2][0], i.corners[2][1]],
                [i.corners[3][0], i.corners[3][1]],
            ])


def main(args=None):
    rclpy.init(args=args)

    draw_corners = DrawCorners()

    rclpy.spin(draw_corners)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    draw_corners.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
