import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion, quaternion_from_euler


assert rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, Quaternion, Point, Pose
from nav_msgs.msg import OccupancyGrid, Odometry
from .utils import LineTrajectory
# from tf_transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R

from scipy.ndimage import grey_dilation
import numpy as np
import heapq, time

class stateMachineInput(Node):
    """
    Based on current position and found/reached goals, outputs a new start/goal point to plan between
    """
    def __init__(self):
        self.declare_parameter('sm_start_pose_topic', "default")
        self.declare_parameter('clicked_point', "default") 
        self.declare_parameter('odom_topic', "default")

        self.start_pose_topic = self.get_parameter('sm_start_pose_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.banana_topic = self.get_parameter('clicked_point').get_parameter_value().string_value

        self.start_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.start_pose_topic,
            self.start_pose_cb,
            10,
        )

        self.odom_sub = self.create_subscription(Odometry,
            self.odom_topic,
            self.odom_cb,
            1,
        )

        self.banana_point_sub = self.create_subscription(
            PoseArray,
            self.banana_topic,
            self.banana_point_cb,
            10,
        )

        self.initial_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10
        )
        
        self.goal_pub = self.create_publisher(
            PoseStamped,
            "/goal_pose",
            1
        )

        self.start_pose = None
        self.current_pose = None
        self.current_goal_pose = None
        self.banana_points = []
        self.goals_reached = [False, False]
    
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

        self.start_pose = np.array([x, y, theta])
        self.get_logger().info(f"Initial Pose Received")
    
    def odom_cb(self, odometry_msg):
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

        # self.current_pose = car

        # Check if at goal
        if np.linalg.norm(self.current_goal_pose - self.current_pose) < 0.25:
            if self.goals_reached[1]:
                self.current_goal_pose = self.start_pose
            elif self.goals_reached[0]:
                self.current_goal_pose = self.banana_points[1]  
            self.replan()         
        
    def banana_point_cb(self, msg: Point):
       # Extract the first two pose positions and convert to numpy arrays
        self.banana_points = [
            np.array([msg.poses[0].position.x, msg.poses[0].position.y]),
            np.array([msg.poses[1].position.x, msg.poses[1].position.y])
        ]

        # Set the current goal pose to the first banana point
        self.current_goal_pose = self.banana_points[0]
        
        self.get_logger().info(f'Banana points set to: {self.banana_points}')

    def replan(self):
        start_x, start_y = self.current_pose
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

        self.get_logger().info("Sent new start and goal poses")
        



if __name__ == "__main__":
    pass