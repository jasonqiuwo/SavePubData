import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray
import serial

class SerialToRos2(Node):

    def __init__(self):
        super().__init__('encoders')
        self.revs_publisher = self.create_publisher(Int32, 'revs', 10)
        self.degrees_publisher = self.create_publisher(Int32, 'degrees', 10)
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.ser.flush()
        self.timer = self.create_timer(0.01, self.read_serial_data)  # 15 Hz

    def read_serial_data(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            if line.startswith("Revolutions:"):
                try:
                    parts = line.split(", ")
                    revs = int(parts[0].split(": ")[1])
                    degrees = int(parts[1].split(": ")[1])

                    revs_msg = Int32()
                    revs_msg.data = revs
                    self.revs_publisher.publish(revs_msg)
                    
                    degrees_msg = Int32()
                    degrees_msg.data = degrees
                    self.degrees_publisher.publish(degrees_msg)

                    self.get_logger().info(f'Published: Revs: {revs}, Degrees: {degrees}')

                except ValueError as e:
                    self.get_logger().error(f"Error parsing line: {line}, Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialToRos2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

