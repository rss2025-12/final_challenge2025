import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
import numpy as np

class ObstaclePublisher(Node): 
    def __init__(self):
        super().__init__("obstacle_publisher")
        self.declare_parameter('width', 100)
        self.declare_parameter('height', 100)
        # self.declare_parameter('resolution', 0.1)
        self.declare_parameter('map_topic', "default")

        self.width = self.get_parameter('width').get_parameter_value().double_value
        self.height = self.get_parameter('height').get_parameter_value().double_value
        # self.resolution = self.get_parameter('resolution').get_parameter_value().double_value

        self.grid_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)
        self.marker_pub = self.create_publisher(Marker, '/obstacle_markers', 10)

        self.click_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.click_callback,
            10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            1
        )

    def map_callback(self, msg):
        self.get_logger().info("Received map")

        self.resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        self.grid = np.array(msg.data, dtype=np.int8).reshape((self.map_height, self.map_width))


    def click_callback(self, msg):
            # Convert clicked point to grid coordinates
            x = int(msg.point.x / self.resolution)
            y = int(msg.point.y / self.resolution)
            
            if 0 <= x < self.width and 0 <= y < self.height:
                self.grid[y, x] = 1

                self.expand_obstacle(x, y)
                
                self.get_logger().info(f'Added obstacle at ({x}, {y})')
            else:
                self.get_logger().warn(f'Clicked point ({msg.point.x}, {msg.point.y}) is outside grid bounds')

    def expand_obstacle(self, x, y):
            """
            Expand the obstacle to create a safety margin
            """
            expansion_radius = 2
            
            for dx in range(-expansion_radius, expansion_radius + 1):
                for dy in range(-expansion_radius, expansion_radius + 1):
                    new_x, new_y = x + dx, y + dy
                    if 0 <= new_x < self.width and 0 <= new_y < self.height:
                        self.grid[new_y, new_x] = 1

    def publish_markers(self):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'
            marker.ns = 'obstacles'
            marker.id = 0
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            
            # Add points for each occupied cell
            for y in range(self.height):
                for x in range(self.width):
                    if self.grid[y, x] == 1:
                        marker.points.append(
                            Point(x=x * self.resolution, y=y * self.resolution, z=0.0)
                        )
            
            marker.scale.x = self.resolution
            marker.scale.y = self.resolution
            self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    grid_manager = ObstaclePublisher()
    rclpy.spin(grid_manager)
    grid_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()