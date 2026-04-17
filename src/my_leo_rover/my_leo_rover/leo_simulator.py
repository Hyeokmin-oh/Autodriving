import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LeoSimulator(Node):
    def __init__(self):
        super().__init__('leo_simulator')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = LaserScan()
        msg.header.frame_id = 'base_link'
        # 가짜 거리 데이터 (모두 1.0m로 설정)
        msg.ranges = [1.0] * 24
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing LiDAR data to /scan')

def main(args=None):
    rclpy.init(args=args)
    node = LeoSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()