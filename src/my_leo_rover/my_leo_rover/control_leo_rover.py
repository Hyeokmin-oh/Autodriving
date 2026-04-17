import pybullet as p
import pybullet_data
import xacro
import os
import time
import random
import math
import cv2
import numpy as np

# --- 1. 경로 설정 및 Xacro 변환 (Leo Rover 로직) ---
current_dir = os.path.dirname(os.path.abspath(__file__))
base_path = os.path.join(current_dir, "leo_common-master", "leo_description")
xacro_file = os.path.join(base_path, "urdf", "leo.urdf.xacro")

try:
    with open(xacro_file, 'r') as f:
        content = f.read()
    content = content.replace("$(find leo_description)", base_path)
    doc = xacro.parse(content)
    xacro.process_doc(doc)
    robot_description = doc.toxml()
    robot_description = robot_description.replace("package://leo_description", base_path)
    
    urdf_file_path = os.path.join(current_dir, "temp_leo.urdf")
    with open(urdf_file_path, "w") as f:
        f.write(robot_description)
except Exception as e:
    print(f"Xacro 변환 오류: {e}")
    exit()

# --- 2. 시뮬레이션 환경 설정 ---
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# --- 3. 미로 생성 및 빌드 ---
def create_maze(width, height):
    maze = [[1] * width for _ in range(height)]
    def walk(x, y):
        maze[y][x] = 0
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        random.shuffle(directions)
        for dx, dy in directions:
            nx, ny = x + dx*2, y + dy*2
            if 0 <= nx < width and 0 <= ny < height and maze[ny][nx] == 1:
                maze[y + dy][x + dx] = 0
                walk(nx, ny)
    walk(1, 1)
    dead_ends = [(x, y) for y in range(1, height-1) for x in range(1, width-1) if maze[y][x] == 0]
    goal_x, goal_y = random.choice(dead_ends) if dead_ends else (width-2, height-2)
    maze[goal_y][goal_x] = 2
    return maze, (goal_x, goal_y)

def build_maze_in_pybullet(maze, wall_height=0.8):
    wall_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, wall_height/2])
    wall_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, wall_height/2], rgbaColor=[0.7, 0.7, 0.7, 1])
    goal_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.4, length=0.01, rgbaColor=[1, 0, 0, 0.5])
    for y, row in enumerate(maze):
        for x, cell in enumerate(row):
            if cell == 1:
                p.createMultiBody(0, wall_shape, wall_visual, [x, y, wall_height/2])
            elif cell == 2:
                p.createMultiBody(0, -1, goal_visual, [x, y, 0.01])

my_maze, goal_coords = create_maze(15, 15)
build_maze_in_pybullet(my_maze)

# --- 4. Leo Rover 로드 및 오도메트리/LiDAR 설정 ---
leo_id = p.loadURDF(urdf_file_path, basePosition=[1, 1, 0.2])
wheel_joints = [2, 3, 5, 6]
for j in wheel_joints:
    p.changeDynamics(leo_id, j, lateralFriction=1.0)

# 오도메트리 전역 변수
total_translation, total_rotation_deg = 0.0, 0.0
last_positions = [0.0, 0.0, 0.0, 0.0]

def get_processed_encoder(robot_id, wheel_indices):
    global total_translation, total_rotation_deg, last_positions
    current_positions = [p.getJointState(robot_id, i)[0] for i in wheel_indices]
    delta_left, delta_right = current_positions[0]-last_positions[0], current_positions[2]-last_positions[2]
    r, L = 0.15, 0.5
    translation = ((delta_left + delta_right) / 2.0) * r
    rotation_rad = ((delta_right - delta_left) * r) / L
    total_translation += translation
    total_rotation_deg += math.degrees(rotation_rad)
    last_positions = current_positions
    return total_translation, total_rotation_deg

def get_4way_lidar(robot_id, num_rays=24, lidar_range=3.0):
    pos, ori = p.getBasePositionAndOrientation(robot_id)
    _, _, yaw = p.getEulerFromQuaternion(ori)
    
    # 이전 디버그 라인 삭제 (실시간 갱신을 위해)
    p.removeAllUserDebugItems()
    
    ray_starts, ray_ends = [], []
    for i in range(num_rays):
        angle = yaw + (2.0 * math.pi * i) / num_rays
        start = [pos[0], pos[1], pos[2] + 0.2]
        end = [pos[0] + lidar_range * math.cos(angle), pos[1] + lidar_range * math.sin(angle), pos[2] + 0.2]
        ray_starts.append(start)
        ray_ends.append(end)
        
        # 방향 확인을 위한 레이 시각화 (전방 섹터는 빨간색, 나머지는 초록색)
        line_color = [1, 0, 0] if (i < 3 or i > 20) else [0, 1, 0]
        p.addUserDebugLine(start, end, line_color)
    
    results = p.rayTestBatch(ray_starts, ray_ends)
    distances = [res[2] * lidar_range for res in results]
    
    return {
        'Front': min(distances[0:3] + distances[21:24]),
        'Left' : min(distances[3:9]),
        'Back' : min(distances[9:15]),
        'Right': min(distances[15:21])
    }

# --- 5. 색상 복구 로직 ---
ORANGE, DARK_GRAY, BLACK, YELLOW, GRAY, BLUE = [1,0.4,0,1], [0.2,0.2,0.2,1], [0.1,0.1,0.1,1], [1,1,0,1], [0.5,0.5,0.5,1], [0,0,0.8,1]
p.changeVisualShape(leo_id, -1, rgbaColor=GRAY)
for i in range(p.getNumJoints(leo_id)):
    link_name = p.getJointInfo(leo_id, i)[12].decode('utf-8').lower()
    if "wheel" in link_name: p.changeVisualShape(leo_id, i, rgbaColor=BLACK)
    elif "rocker" in link_name or "bogie" in link_name: p.changeVisualShape(leo_id, i, rgbaColor=DARK_GRAY)
    elif "camera" in link_name or "lidar" in link_name: p.changeVisualShape(leo_id, i, rgbaColor=YELLOW)
    else: p.changeVisualShape(leo_id, i, rgbaColor=BLUE)

# --- 6. 메인 제어 루프 ---
frame_count = 0
while True:
    p.stepSimulation()
    keys = p.getKeyboardEvents()
    linear_vel, angular_vel = 0, 0
    if keys.get(p.B3G_UP_ARROW, 0) & p.KEY_IS_DOWN: linear_vel = 15
    if keys.get(p.B3G_DOWN_ARROW, 0) & p.KEY_IS_DOWN: linear_vel = -15
    if keys.get(p.B3G_LEFT_ARROW, 0) & p.KEY_IS_DOWN: angular_vel = 15
    if keys.get(p.B3G_RIGHT_ARROW, 0) & p.KEY_IS_DOWN: angular_vel = -15

    for i, joint_id in enumerate(wheel_joints):
        target_v = (linear_vel - angular_vel) if i < 2 else (linear_vel + angular_vel)
        p.setJointMotorControl2(leo_id, joint_id, p.VELOCITY_CONTROL, targetVelocity=target_v, force=15.0)

    cubePos, cubeOrn = p.getBasePositionAndOrientation(leo_id)
    _, _, yaw_deg = np.degrees(p.getEulerFromQuaternion(cubeOrn))
    p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=yaw_deg-90, cameraPitch=-50, cameraTargetPosition=cubePos)

    frame_count += 1
    if frame_count % 20 == 0:
        lidar_data = get_4way_lidar(leo_id)
        trans, rota = get_processed_encoder(leo_id, wheel_joints)
        print("\033[H\033[J")
        print(f"[ODO] Dist: {trans:6.2f}m | Rot: {rota:6.1f}°")
        print(f"[LiDAR] F: {lidar_data['Front']:.2f}m | L: {lidar_data['Left']:.2f}m | R: {lidar_data['Right']:.2f}m | B: {lidar_data['Back']:.2f}m")
        if lidar_data['Front'] < 0.6: print("⚠️ WARNING: OBSTACLE FRONT!")
        
    time.sleep(1./240.)