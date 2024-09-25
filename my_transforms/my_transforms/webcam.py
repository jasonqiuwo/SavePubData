#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, 'webcam_image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish at 10Hz
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(1)  # Capture from default camera

        if not self.cap.isOpened():
            self.get_logger().error("Webcam could not be opened!")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            ros_image.header.frame_id = "world"  # Set frame_id
            self.publisher_.publish(ros_image)
            self.get_logger().info("Publishing webcam image")

    def destroy(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

