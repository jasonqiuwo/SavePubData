import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Float32, Int32
import numpy as np
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from collections import deque
from time import time

class ScanFusion(Node):
    def __init__(self):
        super().__init__('scan_fusion')
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.encoder_sub = self.create_subscription(Int32, 'degrees', self.encoder_callback, 10)
        self.pc_pub = self.create_publisher(PointCloud2, 'pointcloud', 10)
        
        self.encoder_buffer = deque(maxlen=200)  # Buffer to store encoder values (assumes 2s worth of data)
        self.scan_count = 0  # Counter to track the number of scans processed
        
    def encoder_callback(self, msg):
        timestamp = time()
        self.encoder_buffer.append((timestamp, msg.data))

    def get_interpolated_angle(self, scan_time):
        if len(self.encoder_buffer) < 1:
            self.get_logger().warn('No encoder data available yet, skipping scan fusion.')
            return None  # Return None if there is no encoder data
        
        if len(self.encoder_buffer) == 1:
            return self.encoder_buffer[-1][1]  # Only one entry, return the latest value
        
        for i in range(len(self.encoder_buffer) - 1):
            t1, angle1 = self.encoder_buffer[i]
            t2, angle2 = self.encoder_buffer[i + 1]
            if t1 <= scan_time <= t2:
                # Linear interpolation between t1 and t2
                ratio = (scan_time - t1) / (t2 - t1)
                return angle1 + ratio * (angle2 - angle1)
        
        return self.encoder_buffer[-1][1]  # Fallback in case no exact match

    def scan_callback(self, msg):
        scan_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        current_angle = self.get_interpolated_angle(scan_time)
        
        if current_angle is None:
            return  # Skip processing if no encoder data is available

        # Convert the current angle to radians and invert it to align scans back to 0 degrees
        current_angle_rad = np.radians(-current_angle)
        
        angle_increment = msg.angle_increment
        points = []
        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * angle_increment
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            z = 0.0
            
            # Rotate according to the current angle from the encoder
            rotated_x = x * np.cos(current_angle_rad) - y * np.sin(current_angle_rad)
            rotated_y = x * np.sin(current_angle_rad) + y * np.cos(current_angle_rad)
            
            points.append([rotated_x, rotated_y, z])
        
        # Convert points to PointCloud2 and publish
        header = msg.header
        header.frame_id = 'cloud'  # or the appropriate frame_id
        pointcloud_msg = create_cloud_xyz32(header, points)
        self.pc_pub.publish(pointcloud_msg)
        
        self.scan_count += 1
        if self.scan_count % 4 == 0:
            # Print success message every 4 scans
            self.get_logger().info('Fusion successful: Published point cloud with %d points' % len(points))

def main(args=None):
    rclpy.init(args=args)
    node = ScanFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

