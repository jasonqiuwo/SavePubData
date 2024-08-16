import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import csv
import struct

class CSVToPointCloud2Publisher(Node):
    def __init__(self):
        super().__init__('csv_to_pointcloud2_publisher')
        self.publisher = self.create_publisher(PointCloud2, '/pointcloud_from_csv', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.csv_file = open('laser_scan_data.csv', 'r')
        self.csv_reader = csv.reader(self.csv_file)
        next(self.csv_reader)  # Skip the header row
        self.data = list(self.csv_reader)
        self.index = 0

    def create_pointcloud2(self, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'laser_frame'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        point_cloud = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            fields=fields,
            is_bigendian=False,
            point_step=12,
            row_step=12 * len(points),
            data=b''.join(points),
            is_dense=True
        )

        return point_cloud

    def timer_callback(self):
        points = []
        for row in self.data:
            x = float(row[9])
            y = float(row[10])
            z = float(row[11])
            point = struct.pack('fff', x, y, z)
            points.append(point)
        
        pointcloud2_msg = self.create_pointcloud2(points)
        self.publisher.publish(pointcloud2_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CSVToPointCloud2Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

