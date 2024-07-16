import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import csv
import struct


def load_lidar_data(filename):
    data = []
    with open(filename, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # skip header
        for row in reader:
            data.append((float(row[0]), float(row[1]), float(row[2])))
    return data


class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'lidar_points', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.lidar_data = load_lidar_data('lidar_data.csv')

    def timer_callback(self):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(self.lidar_data)
        msg.height = 1
        msg.width = len(self.lidar_data)
        msg.is_dense = True

        points_data = []
        for point in self.lidar_data:
            points_data.append(struct.pack('fff', point[0], point[1], point[2]))
        msg.data = b''.join(points_data)

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
