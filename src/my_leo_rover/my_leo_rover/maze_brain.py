import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist  # 로봇 움직임을 위한 메시지
from cv_bridge import CvBridge
import cv2
import numpy as np

class MazeBrain(Node):
    def __init__(self):
        super().__init__('maze_brain')
        self.bridge = CvBridge()
        
        # 1. 시뮬레이터(Maze_Leo_Rover)에 속도 명령을 보낼 통로 개설
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        
        self.get_logger().info("자율주행 두뇌 노드 가동! 전진을 시작합니다.")

    def image_callback(self, msg):
        # 2. 영상 인지 단계
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 빨간색(목표점) 검출 범위
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        red_pixels = np.sum(mask > 0)
        
        # 3. 판단 및 제어 단계
        twist = Twist()
        
        if red_pixels > 500: # 빨간 벽이 가까이 보이면
            self.get_logger().info(f"목표 발견! 정지합니다. (Pixel: {red_pixels})")
            twist.linear.x = 0.0  # 정지
            twist.angular.z = 0.0
        else:
            # 빨간색이 안 보이면 계속 앞으로 전진
            twist.linear.x = 0.3  # 전진 속도
            twist.angular.z = 0.0
            
        # 4. 명령 전송
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MazeBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()