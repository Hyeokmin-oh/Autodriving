import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class MazeBrain(Node):
    def __init__(self):
        super().__init__('maze_brain')
        self.bridge = CvBridge()
        
        # 시뮬레이터에서 쏘는 영상을 구독(Subscribe)합니다.
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.get_logger().info("AI 두뇌 노드가 가동되었습니다. 영상을 기다리는 중...")

    def image_callback(self, msg):
        # 1. ROS 이미지 메시지를 OpenCV 형식으로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 2. 이미지 처리 (예: 빨간색 목표지점 찾기 또는 장애물 감지)
        # 우선 화면을 띄워 영상이 잘 들어오는지 확인합니다.
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 예시: 빨간색 목표지점 검출 (색상 범위 설정)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        # 결과 화면 표시
        cv2.imshow("Robot View", cv_image)
        cv2.imshow("Red Goal Detection", mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MazeBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()