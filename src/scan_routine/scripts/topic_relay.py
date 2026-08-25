#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class TopicRelay(Node):
    def __init__(self):
        super().__init__('scanar_topic_relay')
        self.pub_cam0 = self.create_publisher(Image, '/cam0/image_raw', 10)
        self.pub_cam1 = self.create_publisher(Image, '/cam1/image_raw', 10)
        
        self.sub_left = self.create_subscription(Image, '/camera/left/image_raw', self.left_cb, 10)
        self.sub_infra1 = self.create_subscription(Image, '/camera/infra1/image_raw', self.left_cb, 10)
        self.sub_right = self.create_subscription(Image, '/camera/right/image_raw', self.right_cb, 10)
        self.sub_infra2 = self.create_subscription(Image, '/camera/infra2/image_raw', self.right_cb, 10)
        self.get_logger().info("ScanAR Topic Relay started successfully: mapping infra/left to /cam0, infra/right to /cam1.")

    def left_cb(self, msg):
        self.pub_cam0.publish(msg)

    def right_cb(self, msg):
        self.pub_cam1.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TopicRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
