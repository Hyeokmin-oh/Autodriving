#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist # 속도 명령 수신을 위해 추가
from cv_bridge import CvBridge
import cv2
import pybullet as p
import pybullet_data
import os
import numpy as np

class MazeLeoRover(Node):
    def __init__(self):
        super().__init__('maze_leo_rover')
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.current_vel = Twist()

        # 1. 경로 정의를 가장 먼저 수행합니다!
        self.package_dir = "/root/ros2_ws/src/my_leo_rover/my_leo_rover"

        # 2. 그 다음 PyBullet 설정을 진행합니다.
        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")

        # 3. 이제 정의된 package_dir을 사용하여 URDF를 로드합니다.
        # 미로 로드
        p.loadURDF(os.path.join(self.package_dir, "maze.urdf"), [0, 0, 0], useFixedBase=True)
        
        # 로봇 로드
        urdf_path = os.path.join(self.package_dir, "temp_leo.urdf")
        self.leo_id = p.loadURDF(urdf_path, basePosition=[1, 1, 0.5])

        self.timer = self.create_timer(1.0/20.0, self.control_loop)
        self.get_logger().info("고정 미로 및 로봇 로드 완료!")

    def cmd_callback(self, msg):
        """AI 두뇌의 명령을 수신합니다."""
        self.current_vel = msg

    def control_loop(self):
        p.stepSimulation()
        self.publish_camera_frame()

        # 1. 로봇의 실시간 위치와 회전 정보를 가져옵니다.
        pos, orn = p.getBasePositionAndOrientation(self.leo_id)
        
        # 2. 쿼터니언 회전값을 각도(Yaw)로 변환합니다.
        _, _, yaw = p.getEulerFromQuaternion(orn)
        
        # 3. 메인 시뮬레이터 카메라가 로봇을 따라가도록 설정합니다.
        p.resetDebugVisualizerCamera(
            cameraDistance=2.5,                # 로봇과의 거리 (2.5m)
            cameraYaw=(yaw * 180 / np.pi) - 90,      # 로봇의 진행 방향 반영
            cameraPitch=-90,                   # 위에서 내려다보는 각도
            cameraTargetPosition=pos           # 카메라의 중심점을 로봇 위치로 고정
        )
        
        # AI 명령(cmd_vel)에 따라 바퀴 제어
        # leo_rover 바퀴 조인트 번호 (일반적으로 2,3,5,6)
        l_speed = self.current_vel.linear.x - self.current_vel.angular.z
        r_speed = self.current_vel.linear.x + self.current_vel.angular.z
        
        for j in [2, 3]: # 왼쪽
            p.setJointMotorControl2(self.leo_id, j, p.VELOCITY_CONTROL, targetVelocity=l_speed*20)
        for j in [5, 6]: # 오른쪽
            p.setJointMotorControl2(self.leo_id, j, p.VELOCITY_CONTROL, targetVelocity=r_speed*20)

    def publish_camera_frame(self):
        # 카메라 링크(8번)에서 영상 획득
        cam_state = p.getLinkState(self.leo_id, 8)
        c_p, c_o = cam_state[0], cam_state[1]
        r_m = p.getMatrixFromQuaternion(c_o)
        fwd = [r_m[0], r_m[3], r_m[6]]
        v_m = p.computeViewMatrix(c_p, [c_p[0]+fwd[0], c_p[1]+fwd[1], c_p[2]+fwd[2]], [r_m[2], r_m[5], r_m[8]])
        p_m = p.computeProjectionMatrixFOV(65, 1.0, 0.1, 100.0)
        
        _, _, rgb, _, _ = p.getCameraImage(80, 60, v_m, p_m)
        img = np.reshape(rgb, (60, 80, 4))[:, :, :3].astype(np.uint8)
        msg = self.bridge.cv2_to_imgmsg(img, encoding="rgb8")
        self.image_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MazeLeoRover()
    rclpy.spin(node)
    p.disconnect()
    rclpy.shutdown()

if __name__ == '__main__': main()