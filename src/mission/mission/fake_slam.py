#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from visualization_msgs.msg import Marker

class FakeSLAM(Node):
    def __init__(self):
        super().__init__('fake_slam')

        # --- THE ENVIRONMENT PARAMETER ---
        # Pass values 1 through 10 via your launch file to switch layouts
        self.declare_parameter('env_id', 1)
        self.env_id = self.get_parameter('env_id').value
        
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.goal_pub = self.create_publisher(Marker, '/goal_point', 10)
        self.timer = self.create_timer(0.5, self.publish_map)
        
        self.width = 600   # 60m
        self.height = 600  # 60m
        self.resolution = 0.1 
        
        # Center (Spawn) and Goal pixel coordinates
        # cx, cy (300, 300) = physical map (0.0, 0.0)
        # gx, gy (500, 300) = physical map (20.0, 0.0)
        self.cx, self.cy = 300, 300
        self.gx, self.gy = 500, 300  
        
        # Generate the initial map
        self.generate_environment()
        
        self.get_logger().info(f"FakeSLAM Initialized. Serving Environment #{self.env_id}")

    def generate_environment(self):
        # Read parameter in case it changed
        self.env_id = self.get_parameter('env_id').value
        
        # Initialize map strictly with 1s (walls) instead of -1
        self.map_data = np.full((self.height, self.width), 100, dtype=np.int8)
        
        # Always carve a 6x6 meter Spawn Room and Goal Room (0s)
        self.draw_rect(self.cy - 30, self.cy + 30, self.cx - 30, self.cx + 30, fill=0)
        self.draw_rect(self.gy - 30, self.gy + 30, self.gx - 30, self.gx + 30, fill=0)
        
        # Route logic for the 10 environments
        idx = self.env_id
        
        if idx == 1:
            # Env 1: Simple straight hallway
            self.draw_rect(self.cy - 10, self.cy + 10, self.cx, self.gx, fill=0)
            
        elif idx == 2:
            # Env 2: Wide corridor with a central blocking pillar
            self.draw_rect(self.cy - 40, self.cy + 40, self.cx, self.gx, fill=0)
            self.draw_rect(self.cy - 20, self.cy + 20, self.cx + 80, self.cx + 120, fill=100)
            
        elif idx == 3:
            # Env 3: Upper bypass (C-shape)
            self.draw_rect(self.cy - 100, self.cy, self.cx - 10, self.cx + 10, fill=0)
            self.draw_rect(self.cy - 100, self.cy - 80, self.cx, self.gx, fill=0)
            self.draw_rect(self.gy - 100, self.gy, self.gx - 10, self.gx + 10, fill=0)
            
        elif idx == 4:
            # Env 4: Lower bypass (C-shape)
            self.draw_rect(self.cy, self.cy + 100, self.cx - 10, self.cx + 10, fill=0)
            self.draw_rect(self.cy + 80, self.cy + 100, self.cx, self.gx, fill=0)
            self.draw_rect(self.gy, self.gy + 100, self.gx - 10, self.gx + 10, fill=0)
            
        elif idx == 5:
            # Env 5: S-Curve
            self.draw_rect(self.cy - 10, self.cy + 10, self.cx, self.cx + 80, fill=0)
            self.draw_rect(self.cy - 60, self.cy + 10, self.cx + 70, self.cx + 90, fill=0)
            self.draw_rect(self.cy - 60, self.cy - 40, self.cx + 70, self.cx + 150, fill=0)
            self.draw_rect(self.cy - 60, self.gy + 10, self.cx + 130, self.cx + 150, fill=0)
            self.draw_rect(self.gy - 10, self.gy + 10, self.cx + 130, self.gx, fill=0)
            
        elif idx == 6:
            # Env 6: Dual narrow choices (top and bottom routes)
            self.draw_rect(self.cy - 40, self.cy - 20, self.cx, self.gx, fill=0)
            self.draw_rect(self.cy + 20, self.cy + 40, self.cx, self.gx, fill=0)
            self.draw_rect(self.cy - 40, self.cy + 40, self.cx + 10, self.cx + 30, fill=0) 
            self.draw_rect(self.gy - 40, self.gy + 40, self.gx - 30, self.gx - 10, fill=0)
            
        elif idx == 7:
            # Env 7: Open room with a grid of scattered small obstacles
            self.draw_rect(self.cy - 60, self.cy + 60, self.cx, self.gx, fill=0)
            for ox in range(self.cx + 40, self.gx - 30, 30):
                for oy in range(self.cy - 40, self.cy + 50, 30):
                    self.draw_rect(oy - 5, oy + 5, ox - 5, ox + 5, fill=100)
                    
        elif idx == 8:
            # Env 8: Zig-zag diagonal steps
            self.draw_rect(self.cy - 15, self.cy + 15, self.cx, self.cx + 50, fill=0)
            self.draw_rect(self.cy - 45, self.cy + 15, self.cx + 40, self.cx + 70, fill=0)
            self.draw_rect(self.cy - 45, self.cy - 15, self.cx + 40, self.cx + 120, fill=0)
            self.draw_rect(self.cy - 45, self.cy + 15, self.cx + 110, self.cx + 140, fill=0)
            self.draw_rect(self.cy - 15, self.cy + 15, self.cx + 110, self.gx, fill=0)

        elif idx == 9:
            # Env 9: Massive open arena
            self.draw_rect(self.cy - 150, self.cy + 150, self.cx, self.gx, fill=0)
            
        elif idx == 10:
            # Env 10: Arena with a massive dividing wall forcing edge navigation
            self.draw_rect(self.cy - 100, self.cy + 100, self.cx, self.gx, fill=0)
            self.draw_rect(self.cy - 70, self.cy + 70, self.cx + 80, self.cx + 120, fill=100)

        else:
            self.get_logger().warn(f"Unknown env_id {idx}. Defaulting to empty field.")
            self.draw_rect(self.cy - 50, self.cy + 50, self.cx, self.gx, fill=0)

    def draw_rect(self, y1, y2, x1, x2, fill=0):
        # Bound limits so we don't index outside the numpy array
        y1, y2 = max(0, int(y1)), min(self.height, int(y2))
        x1, x2 = max(0, int(x1)), min(self.width, int(x2))
        self.map_data[y1:y2, x1:x2] = fill
    
    def publish_goal_marker(self):
        """Creates a bright gold sphere at the 15.0m mark"""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal'
        marker.id = 1
        marker.type = Marker.SPHERE # A spherical point
        marker.action = Marker.ADD

        # The universal goal point (World Coordinates)
        marker.pose.position.x = 20.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5 # Hover half a meter above ground

        marker.pose.orientation.w = 1.0

        # Make it 1 meter wide so it is highly visible
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # Color: Bright Gold
        marker.color.r = 1.0
        marker.color.g = 0.84
        marker.color.b = 0.0
        marker.color.a = 1.0 # Fully solid

        self.goal_pub.publish(marker)

    def publish_map(self):
        # Dynamic parameter checking so you can switch maps without restarting the node!
        current_idx = self.get_parameter('env_id').value
        if current_idx != self.env_id:
            self.env_id = current_idx
            self.generate_environment()
            self.get_logger().info(f"Switched to Environment #{self.env_id}")
            
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        
        # Standard origin mapping
        msg.info.origin.position.x = - (self.width * self.resolution) / 2.0
        msg.info.origin.position.y = - (self.height * self.resolution) / 2.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self.map_data.flatten().tolist()
        
        self.map_pub.publish(msg)
        self.publish_goal_marker()

def main(args=None):
    rclpy.init(args=args)
    node = FakeSLAM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()