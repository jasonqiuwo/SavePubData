import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Int32
import numpy as np
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from collections import deque
from threading import Timer
import time
import csv
import os

class ScanFusion(Node):
    def __init__(self):
        super().__init__('scan_fusion')
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.encoder_sub = self.create_subscription(Int32, 'degree', self.encoder_callback, 10)
        self.pc_pub = self.create_publisher(PointCloud2, 'pointcloud', 10)
        
        self.encoder_buffer = deque(maxlen=1000)
        self.scan_count = 0
        self.pointcloud_data = deque(maxlen=100)
        self.last_save_time = time.time()
        self.save_interval = 4
        
        # Parameters similar to MATLAB script
        self.lidar_range = 5.0  # Maximum range in meters
        self.pivot_angle_deg = 30.0
        self.pivot_angle_rad = np.radians(self.pivot_angle_deg)
        self.rotation_speed_rpm = 60.0
        self.rotation_speed_rad_per_sec = np.radians(360.0 * self.rotation_speed_rpm / 60.0)
        self.scans_per_second = 15.0
        self.angular_resolution_deg = 0.33
        self.angular_resolution_rad = np.radians(self.angular_resolution_deg)

        # Precompute cos and sin of the pivot angle
        self.cos_pivot = np.cos(self.pivot_angle_rad)
        self.sin_pivot = np.sin(self.pivot_angle_rad)
        
        self.csv_filename = 'pointcloud_data.csv'
        if os.path.exists(self.csv_filename):
            os.remove(self.csv_filename) 

    def save_to_csv(self):
        with open(self.csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['X', 'Y', 'Z'])
            for points in self.pointcloud_data:
                for point in points:
                    writer.writerow(point)
                    
    def encoder_callback(self, msg):
        timestamp = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        self.encoder_buffer.append((timestamp, msg.data))

    def scan_callback(self, msg):
        scan_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if len(self.encoder_buffer) < 1:
            self.get_logger().warn('No encoder data available yet, skipping scan fusion.')
            return

        # Get the latest encoder value
        _, current_angle = self.encoder_buffer[-1]

        # Convert the current angle to radians
        current_angle_rad = np.radians(-current_angle)
        
        angle_increment = msg.angle_increment
        points = []
        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max:
                continue
                
            angle = msg.angle_min + i * angle_increment
            x_local = r * np.cos(angle)
            y_local = r * np.sin(angle)
            
            # Apply pivot inclination
            x_pivot = x_local
            y_pivot = y_local * self.cos_pivot
            z_pivot = y_local * self.sin_pivot  # Adjusting for inclination on Z-axis
            
            # Rotate according to the current angle from the encoder
            x_global = x_pivot * np.cos(current_angle_rad) - y_pivot * np.sin(current_angle_rad)
            y_global = x_pivot * np.sin(current_angle_rad) + y_pivot * np.cos(current_angle_rad)
            z_global = z_pivot
            
            points.append([x_global, y_global, z_global])
        
        self.pointcloud_data.append(points)
        
        # Convert points to PointCloud2 and publish
        header = msg.header
        header.frame_id = 'cloud'
        pointcloud_msg = create_cloud_xyz32(header, points)
        self.pc_pub.publish(pointcloud_msg)
        
        self.scan_count += 1
        if self.scan_count % 1 == 0:
            # Print success message every 4 scans
            self.get_logger().info('Fusion successful: Published point cloud with %d points' % len(points))
            
        if time.time() - self.last_save_time >= self.save_interval:
            self.save_to_csv()
            self.last_save_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = ScanFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

