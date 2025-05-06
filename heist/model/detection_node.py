import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from .detector import Detector
from std_msgs.msg import Int32
import numpy as np
import cv2
from std_msgs.msg import Bool


class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector")
        self.detector = Detector()
        #self.publisher = None #TODO
        self.publisher = self.create_publisher(Image, "/detector/output_image", 1)
        self.traffic_light_pub = self.create_publisher(Bool, "/traffic_light_seen", 1)
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 1)
        self.banana_id_sub = self.create_subscription(Int32, "/banana_id", self.banana_id_cb, 1)
        # self.banana_sub = self.create_subscription(Int32, "/banana_id", self.banana_callback, 1)
        self.bridge = CvBridge()

        self.current_banana_id = None
        self.save_filename = 'banana_1.jpg'

        self.get_logger().info("Detector Initialized")



    def callback(self, img_msg):
        # Process image with CV Bridge

        #convert ROS --> CV
        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        #RUN YOLO:
        out = self.detector.predict(image)
        preds = out["predictions"]
        # self.get_logger().info(f'the predicted classes are {preds}')
        #DRAW BOXES:
        traffic_msg = Bool()
        traffic_msg.data = False
        for p in preds:
            if p[-1] == 'traffic light':
                # self.get_logger().info(f'I see a traffic light')
                traffic_msg.data = True
                
            if p[-1] == "banana":
                annotated = self.detector.draw_box(out["original_image"], [p], draw_all =False)
                cv2.imwrite(f'{self.save_filename}', np.array(annotated))

                #convert back to ROS and publish
                out_msg = self.bridge.cv2_to_imgmsg(np.array(annotated), encoding="rgb8")
                self.get_logger().info(f'I see a banana')
                self.publisher.publish(out_msg)
            else: 
                annotated = self.detector.draw_box(out["original_image"], [p], draw_all =True)
                cv2.imwrite(f'test_pic.jpg', np.array(annotated))

                #convert back to ROS and publish
                out_msg = self.bridge.cv2_to_imgmsg(np.array(annotated), encoding="rgb8")
                # self.get_logger().info(f'I do not see a banana')
                # self.publisher.publish(out_msg)

            self.traffic_light_pub.publish(traffic_msg)

    def banana_id_cb(self, id_msg):
        self.current_banana_id = id_msg.data 
        if self.current_banana_id > 0:
            self.save_filename = 'banana_2.jpg'
        

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

