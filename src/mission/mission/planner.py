#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import numpy as np

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.grid = None
        self.map_info = None
        self.goal_x = 20.0  # World X coordinate
        self.goal_y = 0.0   # World Y coordinate

        self.timer = self.create_timer(1.0, self.planning_loop)
        
        self.get_logger().info("Planner Node Started. Waiting for map...")

    def map_callback(self, msg):
        """Converts the 1D OccupancyGrid array into a 2D NumPy array for easy access."""
        self.map_info = msg.info
        
        # Reshape the 1D flat array into a 2D matrix: grid[y][x]
        # 0 = Free Space, 100 = Obstacle
        self.grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

    def get_robot_pose(self):
        """Fetches the current X, Y position of the robot from the TF tree."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except TransformException as ex:
            return None, None

    def world_to_grid(self, x, y):
        """Helper: Converts real-world (x, y) meters to grid matrix indices (col, row)."""
        if not self.map_info: return None, None
        col = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        row = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return col, row

    def grid_to_world(self, col, row):
        """Helper: Converts grid matrix indices (col, row) back to real-world (x, y) meters."""
        if not self.map_info: return None, None
        x = (col * self.map_info.resolution) + self.map_info.origin.position.x
        y = (row * self.map_info.resolution) + self.map_info.origin.position.y
        return x, y

    def publish_path(self, world_waypoints):
        """
        Takes a list of (x, y) tuples in world coordinates and publishes them as a ROS 2 Path.
        """
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for (x, y) in world_waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)

    def planning_loop(self):
        if self.grid is None:
            return

        rx, ry = self.get_robot_pose()
        if rx is None:
            return

        # --- CANDIDATE TODO: IMPLEMENT PATH PLANNING HERE ---
        # 1. Convert rx, ry and goal_x, goal_y to grid coordinates.
        # 2. Search the self.grid array to find a safe path.
        # 3. Convert the safe path back to world coordinates.
        # 4. Call self.publish_path(your_world_path_list)
        # Note that the values in self.grid are 0 for free space and 100 for obstacles. You can use this to check if a cell is safe to traverse.
        # For now, we will publish a dummy direct path to the goal (which will go through walls!).
        
        # Example dummy path directly to goal (Will hit walls!):
        dummy_path = [(rx, ry), (self.goal_x, self.goal_y)]
        self.publish_path(dummy_path)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()