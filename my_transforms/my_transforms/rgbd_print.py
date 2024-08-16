import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import h5py

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('point_cloud_publisher')
        self.publisher = self.create_publisher(PointCloud2, 'point_cloud', 10)

        # Load the HDF5 file
        self.file_path = 'output.h5'  # Replace with your HDF5 file path
        self.file = h5py.File(self.file_path, 'r')
        self.depth_data = self.file['depth'][:]
        self.rgb_data = self.file['rgb'][:]

        # Assuming camera parameters (adjust according to your camera calibration)
        self.fx = 640  # focal length x
        self.fy = 480  # focal length y
        self.cx = 320  # optical center x
        self.cy = 240  # optical center y

        # Setup PointCloud2 message
        self.pc_msg = PointCloud2()
        self.pc_msg.header = Header()
        self.pc_msg.header.frame_id = 'map'  # Adjust frame_id as needed
        self.pc_msg.height = 1
        self.pc_msg.width = self.depth_data.shape[0] * self.depth_data.shape[1]
        self.pc_msg.fields.append(PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1))
        self.pc_msg.fields.append(PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1))
        self.pc_msg.fields.append(PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1))
        self.pc_msg.fields.append(PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1))
        self.pc_msg.is_bigendian = False
        self.pc_msg.point_step = 10
        self.pc_msg.row_step = self.pc_msg.point_step * self.pc_msg.width
        self.pc_msg.is_dense = True  # All points are assumed to be dense

        self.timer = self.create_timer(1.0, self.timer_callback)  # Publish at 1 Hz

    def timer_callback(self):
        # Reshape depth and rgb data to form a point cloud
        depth_flat = self.depth_data.flatten()
        rgb_flat = self.rgb_data.reshape(-1, 3)

        # Create point cloud data
        points = []
        for i in range(len(depth_flat)):
            depth_value = depth_flat[i]
            if depth_value > 0:  # Valid depth value
                x = (i % self.depth_data.shape[1] - self.cx) * depth_value / self.fx
                y = (i // self.depth_data.shape[1] - self.cy) * depth_value / self.fy
                z = depth_value
                rgb = self.rgb_to_uint32(rgb_flat[i])
                points.append([x, y, z, rgb])

        # Fill in point cloud message data
        self.pc_msg.data = np.asarray(points, dtype=np.float32).tobytes()
        self.pc_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish point cloud message
        self.publisher.publish(self.pc_msg)
        self.get_logger().info('Publishing point cloud')

    def rgb_to_uint32(self, rgb):
        return (int(rgb[0]) << 16) | (int(rgb[1]) << 8) | int(rgb[2])

def main(args=None):
    rclpy.init(args=args)
    point_cloud_publisher = PointCloudPublisher()
    rclpy.spin(point_cloud_publisher)
    point_cloud_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
