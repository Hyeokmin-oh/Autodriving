import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image     # ROS 2 이미지 메시지
from cv_bridge import CvBridge       # OpenCV <-> ROS 변환
import cv2
import pybullet as p
import pybullet_data
import xacro
import os
import time
import random
import math
import numpy as np
from collections import deque

class MazeLeoRover(Node):
    def __init__(self):
        super().__init__('maze_leo_rover')
        
        # 1. ROS 2 통신 설정
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10) # 영상 토픽 발행
        
        # 2. PyBullet 시뮬레이션 초기 설정
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        
        # 경로 설정 (사용자 환경에 맞춘 절대 경로)
        self.current_dir = "/root/ros2_ws/src/my_leo_rover/my_leo_rover"
        self.load_leo_rover()

        # 3. 미로 및 주행 환경 구성
        self.width, self.height = 15, 15
        self.my_maze, self.goal_coords = self.create_maze(self.width, self.height)
        self.path_to_follow = self.solve_maze(self.my_maze, (1, 1), self.goal_coords)
        self.build_maze_in_pybullet(self.my_maze)

        # 4. 주행 상태 변수 초기화
        self.path_idx = 0
        self.goal_reached = False
        self.start_time = time.time()
        self.left_wheels, self.right_wheels = [2, 3], [5, 6]

        # 5. 제어 및 영상 송신 루프 타이머 (30Hz)
        self.timer = self.create_timer(1.0/30.0, self.control_loop)
        self.get_logger().info("카메라 송신 및 자율주행 노드가 활성화되었습니다.")

    def load_leo_rover(self):
        """Xacro를 URDF로 변환하여 로봇 로드"""
        base_path = os.path.join(self.current_dir, "leo_common-master", "leo_description")
        xacro_file = os.path.join(base_path, "urdf", "leo.urdf.xacro")
        
        try:
            with open(xacro_file, 'r') as f:
                content = f.read()
            content = content.replace("$(find leo_description)", base_path)
            doc = xacro.parse(content)
            xacro.process_doc(doc)
            robot_description = doc.toxml().replace("package://leo_description", base_path)
            
            urdf_path = os.path.join(self.current_dir, "temp_leo.urdf")
            with open(urdf_path, "w") as f:
                f.write(robot_description)
            
            self.leo_id = p.loadURDF(urdf_path, basePosition=[1, 1, 0.5])
            for j in [2, 3, 5, 6]:
                p.changeDynamics(self.leo_id, j, lateralFriction=1.0)
        except Exception as e:
            self.get_logger().error(f"로봇 로딩 실패: {e}")

    def create_maze(self, width, height):
        """미로 알고리즘 생성"""
        maze = [[1] * width for _ in range(height)]
        def walk(x, y):
            maze[y][x] = 0
            dirs = [(0, 1), (0, -1), (1, 0), (-1, 0)]
            random.shuffle(dirs)
            for dx, dy in dirs:
                nx, ny = x + dx*2, y + dy*2
                if 0 <= nx < width and 0 <= ny < height and maze[ny][nx] == 1:
                    maze[y + dy][x + dx] = 0
                    walk(nx, ny)
        walk(1, 1)
        goal_pos = (width-2, height-2)
        maze[goal_pos[1]][goal_pos[0]] = 2
        return maze, goal_pos

    def solve_maze(self, maze, start, goal):
        """BFS 기반 최단 경로 탐색"""
        queue = deque([([start], start)])
        visited = {start}
        while queue:
            path, (x, y) = queue.popleft()
            if (x, y) == goal: return path
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < len(maze[0]) and 0 <= ny < len(maze) and maze[ny][nx] != 1 and (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((path + [(nx, ny)], (nx, ny)))
        return []

    def build_maze_in_pybullet(self, maze):
        """PyBullet 월드에 미로 구축"""
        wall_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.25])
        wall_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.25], rgbaColor=[0.7, 0.7, 0.7, 1])
        goal_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.4, length=0.01, rgbaColor=[1, 0, 0, 0.5])
        for y, row in enumerate(maze):
            for x, cell in enumerate(row):
                if cell == 1: p.createMultiBody(0, wall_shape, wall_visual, [x, y, 0.25])
                elif cell == 2: p.createMultiBody(0, -1, goal_visual, [x, y, 0.01])

    def publish_camera_frame(self):
        """카메라 데이터를 ROS 2 메시지로 송신"""
        cam_state = p.getLinkState(self.leo_id, 8)
        c_p, c_o = cam_state[0], cam_state[1]
        r_m = p.getMatrixFromQuaternion(c_o)
        fwd = [r_m[0], r_m[3], r_m[6]]
        v_m = p.computeViewMatrix(c_p, [c_p[0]+fwd[0], c_p[1]+fwd[1], c_p[2]+fwd[2]], [r_m[2], r_m[5], r_m[8]])
        p_m = p.computeProjectionMatrixFOV(65, 1.0, 0.1, 100.0)
        
        # 320x240 해상도 렌더링
        _, _, rgb_img, _, _ = p.getCameraImage(320, 240, v_m, p_m)
        
        # 이미지 데이터 변환 및 발행
        img_array = np.reshape(rgb_img, (240, 320, 4))[:, :, :3]
        img_bgr = cv2.cvtColor(img_array.astype(np.uint8), cv2.COLOR_RGB2BGR)
        msg = self.bridge.cv2_to_imgmsg(img_bgr, encoding="bgr8")
        self.image_pub.publish(msg)

    def control_loop(self):
        """메인 제어 및 물리 업데이트 루프"""
        p.stepSimulation()
        self.publish_camera_frame()

        if self.goal_reached:
            for j in self.left_wheels + self.right_wheels:
                p.setJointMotorControl2(self.leo_id, j, p.VELOCITY_CONTROL, targetVelocity=0, force=30.0)
            return

        pos, orn = p.getBasePositionAndOrientation(self.leo_id)
        _, _, yaw = p.getEulerFromQuaternion(orn)

        if self.path_idx < len(self.path_to_follow):
            target_node = self.path_to_follow[self.path_idx]
            dx, dy = target_node[0] - pos[0], target_node[1] - pos[1]
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist < 0.3:
                self.path_idx += 1
                return

            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - yaw
            while angle_diff > math.pi: angle_diff -= 2*math.pi
            while angle_diff < -math.pi: angle_diff += 2*math.pi

            linear_vel, angular_vel = (20.0, angle_diff * 5.0) if abs(angle_diff) < 0.2 else (0.0, 5.0 if angle_diff > 0 else -5.0)
            
            l_speed, r_speed = linear_vel - angular_vel, linear_vel + angular_vel
            for j in self.left_wheels: p.setJointMotorControl2(self.leo_id, j, p.VELOCITY_CONTROL, targetVelocity=l_speed, force=30.0)
            for j in self.right_wheels: p.setJointMotorControl2(self.leo_id, j, p.VELOCITY_CONTROL, targetVelocity=r_speed, force=30.0)

            if dist < 0.4 and target_node == self.goal_coords:
                self.goal_reached = True
                self.get_logger().info("목적지에 도착했습니다!")

        # 카메라 뷰 추적
        p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=yaw*180/math.pi-90, cameraPitch=-50, cameraTargetPosition=pos)

def main(args=None):
    rclpy.init(args=args)
    node = MazeLeoRover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()