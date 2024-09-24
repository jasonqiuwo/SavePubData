import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import time
import math
import re
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from sensor_msgs_py import point_cloud2
import numpy as np
import open3d as o3d
import os

class LiDARRotationTFPublisher(Node):
    def __init__(self):
        super().__init__('lidar_rotation_tf_publisher')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.encoder_sub = self.create_subscription(
            String,
            '/combined_message',
            self.encoder_callback,
            10)
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  
            self.laser_scan_callback,
            10)
        
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2,
            '/adjusted_pointcloud',
            10
        )

        self.rotation_angle = 0.0  # Current rotation angle in degrees
        self.last_encoder_time = time.time()  # Timestamp for the last encoder reading

        self.pivot_angle_deg = 30
        self.pivot_angle_rad = np.radians(self.pivot_angle_deg)

        self.cos_pivot = np.cos(self.pivot_angle_rad)
        self.sin_pivot = np.sin(self.pivot_angle_rad)

    def encoder_callback(self, msg):
        try:
            data = msg.data
            match = re.match(r"Encoder value: (\d+) degrees, Timestamp: (\d+\.\d+)", data)
            if match:
                degree = float(match.group(1))
                timestamp = float(match.group(2))

                # Update the rotation angle
                self.rotation_angle = degree
                
                # Broadcast the updated transform
                self.broadcast_transform(self.rotation_angle)
            else:
                self.get_logger().warn(f'Failed to parse message: {data}')
        except Exception as e:
            self.get_logger().error(f'Error in processing message: {e}')

    def broadcast_transform(self, rotation_degrees):
        # Create a TransformStamped message to broadcast
        t = TransformStamped()

        # Set header information
        # Set header information
        current_time = self.get_clock().now()

        # Add a small delay (e.g., 50ms) to avoid extrapolation issues
        delay_duration_ns = int(1 * 1e6)  # 5 ms in nanoseconds
        delayed_time_ns = current_time.nanoseconds - delay_duration_ns

        # Calculate delayed seconds and nanoseconds
        delayed_sec = delayed_time_ns // int(1e9)  # Convert to seconds (int)
        delayed_nanosec = delayed_time_ns % int(1e9)  # Remaining nanoseconds

        # Ensure both sec and nanosec fields are properly set
        delayed_time = Time(sec=int(delayed_sec), nanosec=int(delayed_nanosec))

        t.header.stamp = delayed_time

        t.header.frame_id = 'base_link'  # Parent frame
        t.child_frame_id = 'cloud'  # LiDAR frame

        pivot_angle_deg = 25.0
        pivot_angle_rad = math.radians(pivot_angle_deg)
        interpolated_angle_rad = math.radians(-rotation_degrees)

        # Set translation (if LiDAR is rotating around a fixed point, translation is zero)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # For the x-axis: Transformation based on the encoder's rotation (interpolated_angle_rad)
        q_pivot_x = 0.0
        q_pivot_y = 0.0
        q_pivot_z = math.sin(interpolated_angle_rad / 2)
        q_pivot_w = math.cos(interpolated_angle_rad / 2)

        # For the pivot (inclination) in y-axis (tilt):
        q_encoder_x = math.sin(pivot_angle_rad / 2)
        q_encoder_y = 0.0
        q_encoder_z = 0.0
        q_encoder_w = math.cos(pivot_angle_rad / 2)

        # Multiply the quaternion of rotation with the quaternion of pivot
        t.transform.rotation.x = (q_pivot_w * q_encoder_x) + (q_encoder_w * q_pivot_x) + (q_pivot_y * q_encoder_z) - (q_pivot_z * q_encoder_y)
        t.transform.rotation.y = (q_pivot_w * q_encoder_y) + (q_encoder_w * q_pivot_y) + (q_pivot_z * q_encoder_x) - (q_pivot_x * q_encoder_z)
        t.transform.rotation.z = (q_pivot_w * q_encoder_z) + (q_encoder_w * q_pivot_z) + (q_pivot_x * q_encoder_y) - (q_pivot_y * q_encoder_x)
        t.transform.rotation.w = (q_pivot_w * q_encoder_w) - (q_pivot_x * q_encoder_x) - (q_pivot_y * q_encoder_y) - (q_pivot_z * q_encoder_z)
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

    def laser_scan_callback(self, msg):
        pointcloud_msg = self.laserscan_to_pointcloud2(msg)

        xyz_array = self.pointcloud2_to_xyz_array(pointcloud_msg)
        
        self.pointcloud_publisher.publish(pointcloud_msg)
        self.save_to_pcd(xyz_array)

    def laserscan_to_pointcloud2(self, scan):
        # Assume scan is a LaserScan message
        points = []
        angle = scan.angle_min

        for i, r in enumerate(scan.ranges):
            if r < scan.range_min or r > scan.range_max:
                continue

            angle += scan.angle_increment

            # Calculate the x, y, z coordinates of the point
            x_local = r * np.cos(angle)
            y_local = r * np.sin(angle)
            z_local = y_local * self.sin_pivot / 2

            # Apply pivot inclination
            x_pivot = x_local
            y_pivot = y_local * self.cos_pivot
            z_pivot = y_local * self.sin_pivot  # Adjusting for inclination on Z-axis

            # Append point (x, y, z) to the list
            points.append([x_local, y_local, z_local])

        # Create the PointCloud2 message
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        header = scan.header

        cloud_data = np.array(points, dtype=np.float32).tostring()

        pointcloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=12,  # 3 fields * 4 bytes each
            row_step=12 * len(points),
            data=cloud_data
        )

        return pointcloud_msg


    def pointcloud2_to_xyz_array(self, cloud_msg):
        """ Converts a ROS PointCloud2 message to a numpy array. """
        # Extract point cloud data to a numpy array
        pc = []
        for point in point_cloud2.read_points(cloud_msg, skip_nans=True):
            pc.append([point[0], point[1], point[2]])
        return np.array(pc)

    def save_to_pcd(self, xyz_array):
        # Create an Open3D point cloud object
        point_cloud = o3d.geometry.PointCloud()
        # Assign the xyz coordinates for the new points
        point_cloud.points = o3d.utility.Vector3dVector(xyz_array)

        # Define the PCD file path
        pcd_filename = '/home/babynuc/tf_ws/src/adjusted_scan.pcd'  # Change to your desired path

        # Load existing point cloud if the file exists
        if os.path.exists(pcd_filename):
            existing_pcd = o3d.io.read_point_cloud(pcd_filename)
            # Combine the new points with the existing points
            combined_points = np.vstack((np.asarray(existing_pcd.points), np.asarray(point_cloud.points)))
            # Create a new point cloud with the combined points
            combined_point_cloud = o3d.geometry.PointCloud()
            combined_point_cloud.points = o3d.utility.Vector3dVector(combined_points)
            
            # Save the combined point cloud back to the PCD file
            o3d.io.write_point_cloud(pcd_filename, combined_point_cloud)
        else:
            # If the file does not exist, save the new point cloud directly
            o3d.io.write_point_cloud(pcd_filename, point_cloud)
   

def main(args=None):
    rclpy.init(args=args)
    node = LiDARRotationTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
