import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import struct
import math

class FrameTester(Node):
    def __init__(self):
        super().__init__('frame_tester')
        self.sub_odom = self.create_subscription(Odometry, '/aft_mapped_to_init', self.odom_cb, 10)
        self.sub_cloud = self.create_subscription(PointCloud2, '/cloud_registered', self.cloud_cb, 10)
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        print("Ready. Rotate scanner and observe centroid vs pose.")

    def odom_cb(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.pose_yaw = math.atan2(siny_cosp, cosy_cosp)

    def cloud_cb(self, msg):
        num_points = msg.width * msg.height
        if num_points == 0: return
        fmt = 'fff'
        stride = max(1, num_points // 500)
        data = msg.data
        sum_x = 0
        sum_y = 0
        count = 0
        for i in range(0, len(data) - 12, msg.point_step * stride):
            x, y, z = struct.unpack_from(fmt, data, i)
            if not math.isnan(x) and not math.isnan(y):
                sum_x += x
                sum_y += y
                count += 1
        if count > 0:
            cx = sum_x / count
            cy = sum_y / count
            print(f"Pose (x={self.pose_x:+.2f}, y={self.pose_y:+.2f}, yaw={math.degrees(self.pose_yaw):+06.1f}deg) | Cloud Centroid (x={cx:+.2f}, y={cy:+.2f})")

def main():
    rclpy.init()
    node = FrameTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
