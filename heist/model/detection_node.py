import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from .detector import Detector
import numpy as np
import cv2


class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector")
        self.detector = Detector()
        #self.publisher = None #TODO
        self.publisher = self.create_publisher(Image, "/detector/output_image", 1)
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 1)
        self.bridge = CvBridge()

        self.get_logger().info("Detector Initialized")



    def callback(self, img_msg):
        # Process image with CV Bridge

        #convert ROS --> CV
        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        #RUN YOLO:
        out = self.detector.predict(image)
        preds = out["predictions"]

        #DRAW BOXES:
        annotated = self.detector.draw_boxes(out["original_image"], preds, draw_all =True)
        cv2.imwrite('screenshot.jpg', annotated)

        #convert back to ROS and publish
        out_msg = self.bridge.cv2_to_imgmsg(np.array(annotated), encoding="rgb8")
        self.publisher.publish(out_msg)


        #TODO: 

def main(args=None):
    rclpy.init(args=args)
    detector = DetectorNode()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()

# image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
# cv2.imwrite('screenshot.jpg', image)
# if is_light:
#     cv2.imwrite('screenshot.jpg', image)

