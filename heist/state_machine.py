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
        self.declare_parameter('sm_start_pose_topic', "default")
        self.declare_parameter('clicked_point_topic', "default") 
        self.declare_parameter('odom_topic', "default")

        self.start_pose_topic = self.get_parameter('sm_start_pose_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.banana_topic = self.get_parameter('clicked_point_topic').get_parameter_value().string_value
        self.marker_pub = self.create_publisher(Marker, '/bananaaa', 10)
        self.banana_id_pub = self.create_publisher(Int32, '/banana_id', 1)
        self.back_up_pub = self.create_publisher(Bool, '/back_up', 1)
        # NEW CODE
        self.detection_sub = self.create_subscription(Image, '/detector/output_image', self.detection_callback, 1)
        self.goal_number = 0

        # self.get_logger().info(f'THE ODOM TOPIC IS {self.odom_topic}')

        self.start_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.start_pose_topic,
            self.start_pose_cb,
            10,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
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
            "/sm_initialpose",
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
        self.goal_found_time = None
        # self.reversed = False
        self.reverse_condition = False
        self.saw_banana = False
    
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

        self.start_pose = np.array([x, y]) #, theta])
        self.current_pose = np.array([x, y])
        self.replan()
        self.get_logger().info(f"Initial Pose Received")
    
    def odom_cb(self, odometry_msg, goal_rad=1.0):
        if self.current_goal_pose is None:
            return
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

        back_up_msg = Bool()
        back_up_msg.data = False

        if self.reverse_condition:
            back_up_msg = Bool()
            back_up_msg.data = True

        self.back_up_pub.publish(back_up_msg)

        # check if we are near the goal, and how long we have waited
        # self.get_logger().info(f'Distance from goal: {np.linalg.norm(self.current_goal_pose - self.current_pose)}')
        near_goal = (np.linalg.norm(self.current_goal_pose - self.current_pose) < goal_rad)
        waited = False

        if near_goal and self.goal_found_time is None and self.saw_banana:
            self.get_logger().info(f'Updating the goal found time')
            self.goal_found_time = self.get_clock().now().nanoseconds
        elif self.goal_found_time is not None:
            self.get_logger().info(f'Already have the goal found time, checking wait condition')
            waited = ((self.get_clock().now().nanoseconds - self.goal_found_time) > 5*1e9)
            self.get_logger().info(f'Wait condition is {waited}')

        if self.goal_number == 0:
            if near_goal and waited:
                self.get_logger().info(f'Reversing')
                self.reverse_condition = True

        elif self.goal_number == 1:
            if near_goal and waited:
                self.get_logger().info(f'Reversing')
                self.reverse_condition = True

        if self.goal_number == 0:
            if self.reverse_condition and self.far_from_goal():
                self.get_logger().info(f'Replanning for banana 2')
                self.current_goal_pose = self.banana_points[1]
                self.goal_number += 1

                msg = Int32()
                msg.data = self.goal_number
                self.banana_id_pub.publish(msg)

                self.goal_found_time = None
                self.reverse_condition = False
                self.saw_banana = False
                self.replan()

        elif self.goal_number == 1:
            if self.reverse_condition and self.far_from_goal():
                self.get_logger().info(f'Replanning to goal')
                self.current_goal_pose = self.start_pose
                self.goal_number += 1

                msg = Int32()
                msg.data = self.goal_number
                self.banana_id_pub.publish(msg)
                
                self.goal_found_time = None
                self.reverse_condition = False
                self.saw_banana = False
                self.replan()

        elif self.goal_number == 2:
            pass

        # self.replan()

        return 
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
    def detection_callback(self, img_msg, goal_rad=2.0):
        near_goal = (np.linalg.norm(self.current_goal_pose - self.current_pose) < goal_rad)
        if near_goal:
            self.saw_banana = True

    # def back_up(self):
    #   pass

    def far_from_goal(self, goal_rad=2.5):
        return (np.linalg.norm(self.current_goal_pose - self.current_pose) > goal_rad)


    def banana_point_cb(self, msg: Point):
       # Extract the first two pose positions and convert to numpy arrays
        if len(self.banana_points) == 2:
            return
        self.banana_points = [
            np.array([msg.poses[0].position.x, msg.poses[0].position.y]),
            np.array([msg.poses[1].position.x, msg.poses[1].position.y])
        ]

        # Set the current goal pose to the first banana point
        self.current_goal_pose = self.banana_points[0]
        self.publish_markers()
        # self.replan()
        
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

    def publish_markers(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'banana:)'
        marker.id = 0
        # marker.type = Marker.POINTS
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