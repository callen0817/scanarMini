#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import sys

class ImageGrabber(Node):
    def __init__(self, save_path):
        super().__init__('fast_calib_image_grabber')
        self.save_path = save_path
        self.bridge = CvBridge()
        
        # Topic name parameter (default: /image_raw)
        self.declare_parameter('image_topic', '/image_raw')
        # self.declare_parameter('image_topic', '/image_rect')
        topic = self.get_parameter('image_topic').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(Image, topic, self.listener_callback, 10)
        
        # Safety Timeout: Exit if no image received within 10 seconds
        self.timer = self.create_timer(10.0, self.timeout_callback)
        self.get_logger().info(f'Listening on {topic}. Timeout in 10s...')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Ensure directory exists
            os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
            cv2.imwrite(self.save_path, cv_image)
            self.get_logger().info(f'✅ Image saved: {self.save_path}')
            raise SystemExit 
        except Exception as e:
            self.get_logger().error(f'❌ Failed to save: {str(e)}')
            raise SystemExit

    def timeout_callback(self):
        self.get_logger().error('Timed out waiting for image! Check your camera driver.')
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    # Allow passing path as arg, default to home if none
    save_to = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/calib_snap.jpg')
    node = ImageGrabber(save_to)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()