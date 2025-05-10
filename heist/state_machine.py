import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
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
        # self.wall_follow_pub = self.create_publisher(Bool, '/wall_follower_switch', 1)
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

        self.drive_sub = self.create_subscription(AckermannDriveStamped, '/traj_follower_drive', self.drive_pass_cb, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/vesc/high_level/input/nav_1', 10)

        # self.wall_follower_sub = self.create_subscription(AckermannDriveStamped, '/wall_follower_command', self.wall_command_passthrough, 10)

        self.start_pose = None
        self.current_pose = None
        self.current_goal_pose = None
        self.goal_found_time = None
        self.saw_banana = False

        # self.banana_points = []
        # self.goals_reached = [False, False]

        self.goal_number = 0
        self.goal_points = [
            np.array([-4.45, 23.1]), 
            np.array([-5.537989139556885, 25.822851181030273]), # first banana
            np.array([-20.36, 25.78]), # second banana
            np.array([-20.1, 27.0]),
            np.array([-20.5, 31.0]),
            np.array([-19.75, 32.773101806640625]), # THIS IS THE THIRD BANANA, NOT THE SECOND
            # np.array([-20.35629653930664, 25.77571678161621]),
            np.array([-32.0, 33.6]), 
            np.array([-54.5, 31.5]),
            # np.array([-45.0, -1.0]),
            np.array([-55.5, 16.7]),
            np.array([-53.5, 1.5]),
        ] 
        self.current_goal_pose = self.goal_points[0]
        self.reverse = False
        self.done = False
        
        self.get_logger().info("State machine initialized")

    def drive_pass_cb(self, msg):
        if self.done is False:
            self.drive_pub.publish(msg)
        else:
            msg.drive.speed = 0.0
            self.drive_pub.publish(msg)

    def switch_drive_sub(self):
        self.destroy_subscription(self.drive_sub)

        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            "/wall_follower_drive", self.drive_pass_cb, 10)

        self.get_logger().info(f"Subscribing to wall follower")

    # def clicked_points_to_goal_sequence(self, points):
    #     """
    #     convert the clicked banana points to the preset points, and sort them
    #     """
    #     new_points_idx = []
    #     for p in points:
    #         real_p_idx = np.argmin([p-p_b for p_b in self.preset_banana_points])
    #         new_points_idx.append(real_p_idx)
        
    #     new_points_idx.sort()
    #     final_banana_points = [self.preset_banana_points[i] for i in new_points_idx]
    #     return final_banana_points

    def banana_point_cb(self, msg: Point):
        return
        # # Extract the first two pose positions and convert to numpy arrays
        # if len(self.banana_points) >= 3:
        #     return
        # banana_points = []
        # banana_points.append(np.array([msg.poses[0].position.x, msg.poses[0].position.y]))
        # banana_points.append(np.array([msg.poses[1].position.x, msg.poses[1].position.y]))

        # self.goal_points.extend(self.clicked_points_to_goal_sequence(banana_points))

        # self.goal_points += self.intermediate_goals

        # # Set the current goal pose to the first banana point
        # self.current_goal_pose = self.goal_points[0]
        # self.publish_markers()
        # self.get_logger().info(f'Banana points set to: {self.goal_points}')

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

        self.goal_points.append(np.array([x, y]))
        self.start_pose = np.array([x, y])
        self.current_pose = np.array([x, y])
        self.publish_markers()
        self.replan()

        msg = Int32()
        msg.data = 0
        self.banana_id_pub.publish(msg)

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
        distance_to_goal = np.linalg.norm(self.current_goal_pose - self.current_pose)
        near_goal = distance_to_goal < goal_rad

        if near_goal and self.goal_found_time is None:
            self.goal_found_time = self.get_clock().now().nanoseconds
            
        # Intermediate
        if self.goal_number == 0:
            self.handle_intermediate(near_goal, 'Path planning to banana 1')
        # Banana 1
        elif self.goal_number == 1:
            self.handle_banana(near_goal, 'Path planning to banana 2')
        # Banana 2
        elif self.goal_number == 2:
            self.handle_banana(near_goal, f'Path planning to point {self.goal_number + 1}')
        # Middle of Banana 2 doorframe
        elif self.goal_number == 3:
            self.handle_intermediate(near_goal, f'Path planning to point {self.goal_number + 1}')
        # Reposition to approach banana 3
        elif self.goal_number == 4:
            self.handle_intermediate(near_goal, f'Path planning to point {self.goal_number + 1}')
        # Banana 3
        elif self.goal_number == 5:
            self.handle_banana(near_goal, 'Path planning to banana 3')
        # Bottom hallway
        elif self.goal_number == 6:
            self.handle_intermediate(near_goal, f'Path planning to point {self.goal_number + 1}')
        # Vending machine hallway
        elif self.goal_number == 7:
            self.handle_intermediate(near_goal, f'Path planning to point {self.goal_number + 1}')
        # Vending machine hallway right side divot
        elif self.goal_number == 8:
            self.handle_intermediate(near_goal, f'Path planning to point {self.goal_number + 1}')
        # Junction
        elif self.goal_number == 9:
            if near_goal:
                self.goal_number += 1
                self.switch_drive_sub()
                self.get_logger().info("Wall following back to start")
        # Start
        elif self.goal_number == 10:
            near_goal = (np.linalg.norm(self.start_pose - self.current_pose) < 2.5)
            if near_goal:
                self.done = True
                self.goal_found_time = None

                self.get_logger().info("Returned home")

    def handle_intermediate(self, near_goal, message):
        if near_goal: 
            self.goal_number += 1
            self.current_goal_pose = self.goal_points[self.goal_number]
            self.goal_found_time = None
            if self.goal_number != 0:
                self.replan(manual_start_pose=self.goal_points[self.goal_number-1])
            else:
                self.replan()

    def handle_banana(self, near_goal, message):
        if near_goal:
            waited = ((self.get_clock().now().nanoseconds - self.goal_found_time) > 5*1e9)
            if waited:
                # self.get_logger().info("Reversing")
                self.reverse = True

        if self.reverse and self.far_from_goal():
            self.goal_number += 1
            self.current_goal_pose = self.goal_points[self.goal_number]
            self.saw_banana = False
            self.goal_found_time = None
            self.reverse = False
            self.replan(manual_start_pose=self.goal_points[self.goal_number-1])

            msg = Int32()
            msg.data = 1
            self.banana_id_pub.publish(msg)
            
            self.get_logger().info(message)

    def detection_callback(self, img_msg, goal_rad=3.0): # goal dist
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

        for point in self.goal_points:
            marker.points.append(Point(x=point[0], y=point[1], z=0.0))

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
