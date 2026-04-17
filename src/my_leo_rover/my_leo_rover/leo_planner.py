import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LeoPlanner(Node):
    def __init__(self):
        super().__init__('leo_planner')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard LiDAR data! Calculating A*...')
        # 가짜 이동 명령 보내기
        move = Twist()
        move.linear.x = 0.5
        self.publisher_.publish(move)

def main(args=None):
    rclpy.init(args=args)
    node = LeoPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()