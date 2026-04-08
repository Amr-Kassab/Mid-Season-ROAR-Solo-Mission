#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import math

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_path = [] # List of (x, y) tuples
        
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("Controller Node Started. Waiting for path...")

    def path_callback(self, msg):
        """Receives the path from the planner and stores it as a simple list of (x, y) tuples."""
        self.current_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def get_robot_state(self):
        """Fetches current X, Y, and Yaw (Theta) of the robot."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return x, y, yaw
        except TransformException as ex:
            return None, None, None

    def control_loop(self):
        rx, ry, ryaw = self.get_robot_state()
        if rx is None or len(self.current_path) == 0:
            return

        # --- CANDIDATE TODO: IMPLEMENT CONTROL LOGIC HERE ---
        # 1. Look at rx, ry, ryaw and self.current_path.
        # 2. Calculate the required linear velocity (v) and angular velocity (w).
        # 3. Publish them.

        v = 1.0 # Linear velocity (Forward/Back) Default to 1 m/s forward
        w = 0.0 # Angular velocity (Left/Right)

        # Publish the command
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()