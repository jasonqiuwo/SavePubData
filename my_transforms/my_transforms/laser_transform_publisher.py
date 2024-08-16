import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class LaserTransformPublisher(Node):
    def __init__(self):
        super().__init__('laser_transform_publisher')
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_transform)

        self.transformStamped = TransformStamped()
        # self.transformStamped.header.frame_id = 'cloud'
        # self.transformStamped.child_frame_id = 'odom'
        self.transformStamped.header.frame_id = 'cloud'
        self.transformStamped.child_frame_id = 'odom'
        self.transformStamped.transform.translation.x = 1.0
        self.transformStamped.transform.rotation.w = 1.0

    def publish_transform(self):
        self.transformStamped.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.transformStamped)

def main(args=None):
    rclpy.init(args=args)
    transform_publisher = LaserTransformPublisher()
    rclpy.spin(transform_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

