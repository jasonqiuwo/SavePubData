import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Float32, Int32
import numpy as np
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from scipy.stats import norm
import random
from collections import deque

class ParticleFilter:
    def __init__(self, num_particles=100, angle_std_dev=5.0):
        self.num_particles = num_particles
        self.angle_std_dev = angle_std_dev
        self.particles = np.zeros(num_particles)
        self.weights = np.ones(num_particles) / num_particles

    def predict(self, control, control_std_dev):
        self.particles += np.random.normal(control, control_std_dev, self.num_particles)

    def update(self, measurement):
        self.weights *= norm(self.particles, self.angle_std_dev).pdf(measurement)
        self.weights /= np.sum(self.weights)  # Normalize

    def resample(self):
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.0  # Ensure the last value is 1

        # Convert np.linspace to list for compatibility with random.sample
        random_values = random.sample(np.linspace(0, 1, self.num_particles).tolist(), self.num_particles)
        indexes = np.searchsorted(cumulative_sum, random_values)
        self.particles[:] = self.particles[indexes]
        self.weights.fill(1.0 / self.num_particles)

    def estimate(self):
        return np.average(self.particles, weights=self.weights)

class ScanFusion(Node):
    def __init__(self):
        super().__init__('scan_fusion')
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.encoder_sub = self.create_subscription(Int32, 'degrees', self.encoder_callback, 10)
        self.pc_pub = self.create_publisher(PointCloud2, 'pointcloud', 10)
        
        self.encoder_buffer = deque(maxlen=200)
        self.scan_count = 0
        self.pf = ParticleFilter(num_particles=500, angle_std_dev=5.0)

    def encoder_callback(self, msg):
        timestamp = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        self.encoder_buffer.append((timestamp, msg.data))
        
        # Predict phase of the particle filter
        if len(self.encoder_buffer) > 1:
            # Control is the change in angle between the latest two encoder readings
            _, last_angle = self.encoder_buffer[-2]
            _, current_angle = self.encoder_buffer[-1]
            control = current_angle - last_angle
            control_std_dev = 0.1  # Assumed standard deviation for control
            self.pf.predict(control, control_std_dev)
        
        # Update phase of the particle filter
        self.pf.update(msg.data)
        
        # Resample phase of the particle filter
        self.pf.resample()

    def scan_callback(self, msg):
        scan_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if len(self.encoder_buffer) < 1:
            self.get_logger().warn('No encoder data available yet, skipping scan fusion.')
            return

        # Estimate the current angle using the particle filter
        current_angle = self.pf.estimate()

        # Convert the current angle to radians and invert it to align scans back to 0 degrees
        current_angle_rad = np.radians(current_angle)
        
        angle_increment = msg.angle_increment
        points = []
        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max:
                continue
                
            angle = msg.angle_min + i * angle_increment
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            z = 0.0
            
            # Rotate according to the inverse of the current angle from the encoder
            rotated_x = x * np.cos(-current_angle_rad) - y * np.sin(-current_angle_rad)
            rotated_y = x * np.sin(-current_angle_rad) + y * np.cos(-current_angle_rad)
            
            points.append([rotated_x, rotated_y, z])
        
        # Convert points to PointCloud2 and publish
        header = msg.header
        header.frame_id = 'cloud'
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

