import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ObjDetection(Node):
    def __init__(self):
        super().__init__("obj_detection")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, "/camera1/image_raw", self.image_callback, qos_profile_sensor_data
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Webcam Stream", cv_image)
        except Exception as e:
            self.get_logger().error("cv_bridge exception: %s" % e)


def main(args=None):
    rclpy.init(args=args)

    obj_detection = ObjDetection()
    rclpy.spin(obj_detection)

    obj_detection.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
