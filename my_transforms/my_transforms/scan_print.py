import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import csv
import math

class LaserScanToCSV(Node):
    def __init__(self):
        super().__init__('laser_scan_to_csv')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Adjust the topic name as needed
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.csv_file = open('laser_scan_data.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['header_stamp', 'angle_min', 'angle_max', 'angle_increment',
                                  'time_increment', 'scan_time', 'range_min', 'range_max', 
                                  'range', 'x', 'y', 'z'])

    def listener_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for i, range in enumerate(msg.ranges):
            if math.isinf(range) or math.isnan(range):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x = range * math.cos(angle)
            y = range * math.sin(angle)
            z = 0.0  # Assuming 2D lidar, so z is 0
            self.csv_writer.writerow([
                timestamp,
                msg.angle_min,
                msg.angle_max,
                msg.angle_increment,
                msg.time_increment,
                msg.scan_time,
                msg.range_min,
                msg.range_max,
                range,
                x,
                y,
                z
            ])

    def destroy_node(self):
        super().destroy_node()
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToCSV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

