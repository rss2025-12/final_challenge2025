import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, Point
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import Int32, Bool

class StateMachine(Node):
    """
    Based on current position and found/reached goals, outputs a new start/goal point to plan between
    """
    def __init__(self):
        super().__init__("state_machine")
        self.declare_parameter('sm_start_pose_topic', "/initialpose")
        self.declare_parameter('clicked_point_topic', "/shrinkray_part")
        self.declare_parameter('odom_topic', "/pf/pose/odom")

        self.start_pose_topic = self.get_parameter('sm_start_pose_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        self.banana_topic = self.get_parameter('clicked_point_topic').get_parameter_value().string_value
        self.marker_pub = self.create_publisher(Marker, '/bananaaa', 10)
        self.banana_id_pub = self.create_publisher(Int32, '/banana_id', 1)

        self.back_up_pub = self.create_publisher(Bool, '/back_up', 1)
        self.detection_sub = self.create_subscription(Image, '/detector/output_image', self.detection_callback, 1)

        self.start_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.start_pose_topic, self.start_pose_cb, 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic, self.odom_cb, 1)

        self.banana_point_sub = self.create_subscription(
            PoseArray,
            self.banana_topic, self.banana_point_cb, 10)

        self.initial_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/sm_initialpose", 10)

        self.goal_pub = self.create_publisher(
            PoseStamped,
            "/goal_pose", 1)

        self.start_pose = None
        self.current_pose = None
        self.current_goal_pose = None

        self.banana_points = []
        self.goals_reached = [False, False]
        self.saw_banana = False
        self.goal_found_time = None

        self.goal_number = 0
        self.goal_points = [np.array([-6.15, 20.7])]
        self.intermediate_goals = [np.array([-32.0, 34.0]), np.array([-54.6, 25.2]), np.array([-45.0, -1.0])]
        self.reverse_condition = False
        self.reverse = False

    def banana_point_cb(self, msg: Point):
       # Extract the first two pose positions and convert to numpy arrays
        if len(self.banana_points) >= 3:
            return

        self.goal_points.append(np.array([msg.poses[0].position.x, msg.poses[0].position.y]))
        self.goal_points.append(np.array([msg.poses[1].position.x, msg.poses[1].position.y]))

        # Set the current goal pose to the first banana point
        self.current_goal_pose = self.goal_points[0]
        self.publish_markers()
        self.get_logger().info(f'Banana points set to: {self.banana_points}')

    def start_pose_cb(self, pose):
        x = pose.pose.pose.position.x
        y = pose.pose.pose.position.y
        quaternion = [
            pose.pose.pose.orientation.x,
            pose.pose.pose.orientation.y,
            pose.pose.pose.orientation.z,
            pose.pose.pose.orientation.w
        ]
        _, _, theta = R.from_quat(quaternion).as_euler('xyz', degrees=False)

        self.start_pose = np.array([x, y])
        self.current_pose = np.array([x, y])
        self.replan()
        self.get_logger().info(f"Initial path requested")

    def odom_cb(self, odometry_msg, goal_rad=1.0):
        if self.current_goal_pose is None:
            return

        # Current pose
        x = odometry_msg.pose.pose.position.x
        y = odometry_msg.pose.pose.position.y
        quaternion = [
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y,
            odometry_msg.pose.pose.orientation.z,
            odometry_msg.pose.pose.orientation.w
        ]
        _, _, theta = R.from_quat(quaternion).as_euler('xyz', degrees=False)
        self.current_pose = np.array([x, y])

        # Backing up
        back_up_msg = Bool()
        back_up_msg.data = self.reverse
        self.back_up_pub.publish(back_up_msg)

        # Check if we are near the goal
        near_goal = (np.linalg.norm(self.current_goal_pose - self.current_pose) < goal_rad)

        if near_goal and self.goal_found_time is None:
            self.goal_found_time = self.get_clock().now().nanoseconds

        # Intermediate
        if self.goal_number == 0:
            if near_goal:
                self.current_goal_pose = self.goal_points[1]
                self.get_logger().info(f'Path planning to banana 1')
                self.replan()
        # Banana 1
        elif self.goal_number == 1:
            if near_goal:
                waited = ((self.get_clock().now().nanoseconds - self.goal_found_time) > 5*1e9)
                if waited:
                    self.reverse = True

            if self.reverse and self.far_from_goal():
                self.goal_number += 1
                self.current_goal_pose = self.goal_points[2]

                self.saw_banana = False
                self.goal_found_time = None
                self.reverse = False
                self.get_logger().info(f'Path planning to banana 2')
                self.replan()
        # Banana 2
        elif self.goal_number == 2:
            if near_goal:
                waited = ((self.get_clock().now().nanoseconds - self.goal_found_time) > 5*1e9)
                if waited:
                    self.reverse = True

            if self.reverse and self.far_from_goal():
                self.goal_number += 1
                self.current_goal_pose = self.goal_points[3]

                msg = Int32()
                msg.data = self.goal_number
                self.banana_id_pub.publish(msg)

                self.saw_banana = False
                self.goal_found_time = None
                self.reverse = False
                self.get_logger().info(f'Path planning to start')
                self.replan()

    def detection_callback(self, img_msg, goal_rad=0.75): # goal dist
        near_goal = (np.linalg.norm(self.current_goal_pose - self.current_pose) < goal_rad)
        if near_goal:
            self.saw_banana = True

    def far_from_goal(self, goal_rad=1.5):
        return (np.linalg.norm(self.current_goal_pose - self.current_pose) > goal_rad)

    def replan(self, manual_start_pose=None):
        start_x, start_y = self.current_pose
        if manual_start_pose is not None:
            start_x, start_y = manual_start_pose
        theta = np.pi
        goal_x, goal_y = self.current_goal_pose

        # Set initial pose
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.header.frame_id = 'map' #TODO: should this be different?

        initial_pose_msg.pose.pose.position.x = start_x
        initial_pose_msg.pose.pose.position.y = start_y
        initial_pose_msg.pose.pose.position.z = 0.0

        # Orientation (pi radians, facing backwards along the x-axis)
        # Quaternion for a 180-degree rotation (π radians) around the Z-axis
        initial_pose_msg.pose.pose.orientation.x = 0.0
        initial_pose_msg.pose.pose.orientation.y = 0.0
        initial_pose_msg.pose.pose.orientation.z = np.sin(theta / 2)
        initial_pose_msg.pose.pose.orientation.w = np.cos(theta / 2)

        # Publish goal pose
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map' #TODO: should this be different?
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y

        self.initial_pub.publish(initial_pose_msg)
        self.goal_pub.publish(goal_msg)

        self.get_logger().info("Sent start and goal poses")

    def publish_markers(self):
        """
        Publishing banana positions for RViz
        """
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'banana:)'
        marker.id = 0
        marker.type = 8
        marker.action = Marker.ADD

        marker.points.append(Point(x=self.banana_points[0][0], y=self.banana_points[0][1], z=0.0))
        marker.points.append(Point(x=self.banana_points[1][0], y=self.banana_points[1][1], z=0.0))

        marker.scale.x = .5
        marker.scale.y = .5
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    grid_manager = StateMachine()
    rclpy.spin(grid_manager)
    grid_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    # def check_goal_pose(self, goal_rad):
    #     # Check if at goal
    #     self.get_logger().info(f'Distance to goal pose {np.linalg.norm(self.current_goal_pose - self.current_pose)}')
    #     if np.linalg.norm(self.current_goal_pose - self.current_pose) < goal_rad:
    #         self.get_logger().info('At current goal pose')
    #         if self.goals_reached[1]:
    #             self.current_goal_pose = self.start_pose
    #         elif self.goals_reached[0]:
    #             self.current_goal_pose = self.banana_points[1]
    #             self.goals_reached[1] = True
    #         else:
    #             self.goals_reached[0] = True
    #         return True
    #     return False
    # NEW CODE
