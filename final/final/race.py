import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
from cv.race_cv import show_video, record_video, lane_following

class Race(Node):
    def __init__(self):
        super().__init__("race")
        self.declare_parameter('drive_topic', "default")
        self.declare_parameter('drive_speed', 5.0)

        self.drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.drive_speed = self.get_parameter('drive_speed').get_parameter_value().double_value

        # Subscribe to ZED camera RGB frames
        self.image_sub = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

        self.drive_pub = self.create_publisher(AckermannDriveStamped,
                                               self.drive_topic,
                                               1)

        self.get_logger().info("Race node initialized")

    def image_callback(self, image_msg):
        img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # show_video(img)
        record_video(img)

def main(args=None):
    rclpy.init(args=args)
    node = Race()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
