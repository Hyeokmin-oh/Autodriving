import os
import random
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
import heapq
import time

# ==========================================
# 1. 미로 생성 로직 (make_maze_urdf.py 통합)
# ==========================================
def create_maze(w, h):
    maze = [[1] * w for _ in range(h)]
    def walk(x, y):
        maze[y][x] = 0
        dirs = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        random.shuffle(dirs)
        for dx, dy in dirs:
            nx, ny = x + dx*2, y + dy*2
            if 0 <= nx < w and 0 <= ny < h and maze[ny][nx] == 1:
                maze[y+dy][x+dx] = 0
                walk(nx, ny)
    walk(1, 1)
    return maze

def save_to_urdf(maze, filename):
    # 기존 파일이 있으면 삭제 (최적화)
    if os.path.exists(filename):
        os.remove(filename)
        
    urdf_content = '<?xml version="1.0" ?>\n<robot name="maze">\n  <link name="base_link"/>\n'
    wall_count = 0
    for y, row in enumerate(maze):
        for x, cell in enumerate(row):
            if cell == 1:
                wall_count += 1
                urdf_content += f"""
  <link name="wall_{wall_count}">
    <visual>
      <origin xyz="{x} {y} 0.5"/>
      <geometry><box size="1 1 1"/></geometry>
      <material name="gray"><color rgba="0.6 0.6 0.6 1"/></material>
    </visual>
    <collision>
      <origin xyz="{x} {y} 0.5"/>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
  </link>
  <joint name="joint_{wall_count}" type="fixed">
    <parent link="base_link"/>
    <child link="wall_{wall_count}"/>
  </joint>
"""
    urdf_content += "</robot>"
    with open(filename, "w") as f:
        f.write(urdf_content)

# ==========================================
# 2. A* 알고리즘 로직
# ==========================================
class Node:
    def __init__(self, parent=None, position=None):
        self.parent, self.position = parent, position
        self.g = self.h = self.f = 0
    def __lt__(self, other): return self.f < other.f

def astar(maze, start, end):
    start_node, end_node = Node(None, start), Node(None, end)
    open_list, closed_set = [], set()
    heapq.heappush(open_list, start_node)
    
    while open_list:
        current_node = heapq.heappop(open_list)
        closed_set.add(current_node.position)
        
        if current_node.position == end_node.position:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]
        
        (y, x) = current_node.position
        for next_step in [(y-1, x), (y+1, x), (y, x-1), (y, x+1)]:
            if not (0 <= next_step[0] < len(maze) and 0 <= next_step[1] < len(maze[0])) or \
               maze[next_step[0]][next_step[1]] == 1 or next_step in closed_set:
                continue
            child = Node(current_node, next_step)
            child.g = current_node.g + 1
            child.h = abs(child.position[0] - end_node.position[0]) + abs(child.position[1] - end_node.position[1])
            child.f = child.g + child.h
            if any(n for n in open_list if child.position == n.position and child.g > n.g): continue
            heapq.heappush(open_list, child)
    return None

# ==========================================
# 3. 메인 통합 실행부
# ==========================================
# --- [메인 루프: 실시간 갱신 버전] ---
def main():
    GRID_SIZE = 15
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    
    goal_pos = (13, 13) 
    agent_pos = (1, 1)  

    print("실시간 지형 변화 시뮬레이션 시작!")

    try:
        while True:
            # 1. 미로 생성 및 초기화
            my_maze_data = create_maze(GRID_SIZE, GRID_SIZE)
            maze_grid = np.array(my_maze_data)
            maze_grid[goal_pos[0], goal_pos[1]] = 0 
            maze_grid[agent_pos[0], agent_pos[1]] = 0 
            
            # 미로가 생성된 시점 기록
            last_maze_update_time = time.time()
            
            while agent_pos != goal_pos:
                # 2. 현재 위치에서 다시 길 찾기 (리플래닝)
                path = astar(maze_grid, agent_pos, goal_pos)
                
                if not path or len(path) < 2:
                    print("길이 막혔습니다! 강제로 미로를 재생성합니다.")
                    break # 내부 while문을 빠져나가 미로를 새로 만듭니다.

                # 3. 한 칸 이동
                agent_pos = path[1] # path[0]은 현재 위치이므로 path[1]로 이동
                
                # 시각화
                ax.clear()
                ax.imshow(maze_grid, cmap='Greys', origin='lower')
                ax.plot(goal_pos[1], goal_pos[0], 'rs', markersize=15, label='Goal')
                ax.plot(agent_pos[1], agent_pos[0], 'yo', markersize=12, label='Agent')
                
                # 현재 계산된 최신 경로 표시
                px, py = [p[1] for p in path], [p[0] for p in path]
                ax.plot(px, py, color='yellow', linewidth=2, alpha=0.3, linestyle='--')
                
                ax.set_title(f"Dynamic Maze - Goal at {goal_pos}")
                plt.draw()
                plt.pause(0.3) # 이동 속도

                # 4. 핵심 로직: 이동 중에 5초가 지났는지 체크
                current_time = time.time()
                if current_time - last_maze_update_time >= 5.0:
                    print("⚠️ 5초 경과! 미로 지형이 변경됩니다!")
                    break # 현재 경로 이동을 중단하고 상위 while문으로 가서 미로를 새로 만듭니다.

            if agent_pos == goal_pos:
                print("🎉 축하합니다! 목적지에 도달했습니다.")
                ax.set_title("GOAL REACHED!")
                plt.draw()
                plt.pause(3)
                agent_pos = (1, 1) # 다시 시작점으로

    except KeyboardInterrupt:
        print("\n시뮬레이션을 종료합니다.")

if __name__ == "__main__":
    main()