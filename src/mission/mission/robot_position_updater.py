#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
import math

class RobotPositionUpdater(Node):
    def __init__(self):
        super().__init__('robot_position_updater')

        # --- State Variables ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.v = 0.0 # Linear velocity
        self.w = 0.0 # Angular velocity

        self.last_time = self.get_clock().now()

        # --- Subscribers, Publishers, and Broadcasters ---
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/robot_marker', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Update loop at 50Hz (0.02 seconds)
        self.timer = self.create_timer(0.02, self.update_position)

        self.get_logger().info("Robot Position Updater initialized. Listening to /cmd_vel...")

    def cmd_vel_callback(self, msg):
        """Update current velocities based on Twist messages."""
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update_position(self):
        """Integrate velocities to update position, then publish TF and Marker."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 1. Kinematic Integration (Differential Drive / Unicycle Model)
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.w * dt

        # 2. Publish TF Tree (map -> base_link)
        self.publish_tf()

        # 3. Publish Visualization Marker
        self.publish_marker()

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Convert Yaw (theta) to Quaternion for TF
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Set the pose of the marker
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.1 # Slightly above the ground

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(self.theta / 2.0)
        marker.pose.orientation.w = math.cos(self.theta / 2.0)

        # Set the scale of the box (Length, Width, Height in meters)
        marker.scale.x = 0.4  # 40cm long
        marker.scale.y = 0.25 # 25cm wide
        marker.scale.z = 0.15 # 15cm tall

        # Set the color (Red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0 # Alpha (transparency)

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = RobotPositionUpdater()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()