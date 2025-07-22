import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cypiu.modules.yolov4 as yolov4
import numpy as np


class ObjDetection(Node):
    def __init__(self):
        super().__init__("obj_detection")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, "camera1/image_raw", self.image_callback, qos_profile_sensor_data
        )

    def image_callback(self, msg):
        self.get_logger().info("Image Received!")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            image_pp = yolov4.image_preprocess(np.copy(cv_image)).astype(np.float32)
            image_data = image_pp[np.newaxis, ...].astype(np.float32)

            inference = yolov4.make_inference(image_data)
            bbox_info = yolov4.postprocess(inference)
            final_image = yolov4.draw_bbox(image_pp, bbox_info)
            self.get_logger().info("Processing Finished!")
            cv2.imshow("Webcam Stream", final_image)
            cv2.waitKey(1)
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
