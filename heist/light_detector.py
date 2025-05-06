#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation import cd_color_segmentation, detect_apriltags


class LightDetector(Node):
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /redlight (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        super().__init__("light_detector")
        # Subscribe to ZED camera RGB frames
        self.debug = True
        self.lights_pub = self.create_publisher(Bool, "/redlight", 10)
        if self.debug:
            self.debug_pub = self.create_publisher(Image, "/debug_img", 10)
        self.image_sub = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

        ### AprilTag Detection Dictionary ###
        # self.tag_positions = {}
        # for tag_id in range(9):
        #     self.tag_positions[tag_id] = {
        #         'total_center_u': 0,
        #         'total_center_v': 0,
        #         'total_X': 0,
        #         'total_Y': 0,
        #         'count': 0
        #     }

        self.get_logger().info("Light Detector Initialized")

    def image_callback(self, image_msg):
        """
        Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        From your bounding box, take the center pixel on the bottom
        (We know this pixel corresponds to a point on the ground plane)
        publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        convert it to the car frame.
        """

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        bounding_box = cd_color_segmentation(image, "placeholder", False)

        ### AprilTag Detection (Remeber to comment out) ###
        # self.april_tag_distances(detect_apriltags(image))

        (x1, y1), (x2, y2) = bounding_box
        length = x2-x1
        width = y2-y1
        area = length*width
        is_light = area>100

        light_on_and_close = Bool()
        light_on_and_close.data = is_light
        # self.get_logger().info(f'Traffic light detector sees light: {is_light}')
        self.lights_pub.publish(light_on_and_close)

        if self.debug:
            # self.get_logger().info(f"Light detected with area {area} px^2, length/width {length}, {width} px")
            if(not is_light):
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
            debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.debug_pub.publish(debug_msg)

    def april_tag_distances(self, positions):
        for tag in positions:
            tag_id = tag['id']
            u, v = tag['center']
            X, Y = tag['X'], tag['Y']
            tag_position = self.tag_positions[tag_id]
            tag_position['total_center_u'] += u
            tag_position['total_center_v'] += v
            tag_position['total_X'] += X
            tag_position['total_Y'] += Y
            tag_position['count'] += 1

        if self.tag_positions[0]['count'] == 200:
            for tag_id, data in self.tag_positions.items():
                count = data['count']
                if count > 0:
                    avg_center_u = data['total_center_u'] / count
                    avg_center_v = data['total_center_v'] / count
                    avg_X = data['total_X'] / count
                    avg_Y = data['total_Y'] / count

                    print(f"Tag {tag_id} Averaged Center: ({avg_center_u:.2f}, {avg_center_v:.2f})")
                    print(f"Tag {tag_id} Averaged Position: (X: {avg_X:.2f}, Y: {avg_Y:.2f})")


def main(args=None):
    rclpy.init(args=args)
    light_detector = LightDetector()
    rclpy.spin(light_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
