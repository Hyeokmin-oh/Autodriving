import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
import random
import heapq

# --- 1. A* 알고리즘을 위한 Node 클래스 정의 ---
class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0  # 출발점에서 현재까지의 거리
        self.h = 0  # 현재에서 목적지까지의 예상 거리
        self.f = 0  # f = g + h
    
    def __lt__(self, other):
        return self.f < other.f

# --- 2. A* 알고리즘 함수 ---
def astar(maze, start, end):
    # start, end는 (y, x) 형태
    start_node = Node(None, start)
    end_node = Node(None, end)
    
    open_list = []
    closed_set = set()
    
    heapq.heappush(open_list, start_node)
    
    while open_list:
        current_node = heapq.heappop(open_list)
        closed_set.add(current_node.position)
        
        # 목적지 도착 시 경로 재구성
        if current_node.position == end_node.position:
            path = []
            while current_node is not None:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1] # 역순 반환
        
        (y, x) = current_node.position
        # 상하좌우 탐색
        for next_step in [(y-1, x), (y+1, x), (y, x-1), (y, x+1)]:
            # 맵 범위 체크
            if not (0 <= next_step[0] < len(maze) and 0 <= next_step[1] < len(maze[0])):
                continue
            # 벽 체크
            if maze[next_step[0]][next_step[1]] == 1:
                continue
            # 방문 여부 체크
            if next_step in closed_set:
                continue
                
            child = Node(current_node, next_step)
            child.g = current_node.g + 1
            # 맨해튼 거리 휴리스틱
            child.h = abs(child.position[0] - end_node.position[0]) + abs(child.position[1] - end_node.position[1])
            child.f = child.g + child.h
            
            # 이미 탐색 목록에 있는 더 나은 경로는 무시
            if any(open_node for open_node in open_list if child.position == open_node.position and child.g > open_node.g):
                continue
                
            heapq.heappush(open_list, child)
    return None

# --- 3. URDF를 그리드로 변환하는 함수 ---
def urdf_to_grid(urdf_file, grid_size=15):
    grid = np.zeros((grid_size, grid_size))
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    
    for link in root.findall('link'):
        if 'wall' in link.get('name', ''):
            visual = link.find('.//visual/origin')
            if visual is not None:
                xyz = list(map(float, visual.get('xyz').split()))
                x, y = int(xyz[0]), int(xyz[1])
                if 0 <= x < grid_size and 0 <= y < grid_size:
                    grid[y, x] = 1
    return grid

# --- 4. 메인 실행부 ---
def main():
    grid_size = 15
    # URDF 읽기 (파일 경로 확인 필수)
    maze_grid = urdf_to_grid('maze.urdf', grid_size=grid_size)

    # 시작점/도착점 설정
    start_pos_xy = (1, 1) # 시각화용 (x, y)
    start_pos_yx = (1, 1) # 알고리즘용 (y, x)
    
    paths = np.argwhere(maze_grid == 0)
    path_list = [tuple(p) for p in paths if not (p[0] == start_pos_yx[0] and p[1] == start_pos_yx[1])]
    
    if not path_list:
        print("에러: 이동 가능한 경로가 없습니다.")
        return

    goal_pos_yx = random.choice(path_list)
    print(f"시작점: {start_pos_xy}, 도착점: (x:{goal_pos_yx[1]}, y:{goal_pos_yx[0]})")

    # A* 알고리즘 실행
    path = astar(maze_grid, start_pos_yx, goal_pos_yx)

    # 시각화
    plt.figure(figsize=(10, 10))
    plt.imshow(maze_grid, cmap='Greys', origin='lower')

    if path:
        print(f"경로 탐색 성공! 이동 거리: {len(path)-1}")
        px = [p[1] for p in path]
        py = [p[0] for p in path]
        # 경로를 노란색 선으로 표시
        plt.plot(px, py, color='yellow', linewidth=3, label='A* Path', zorder=1)
    else:
        print("경로를 찾을 수 없습니다.")

    # 시작/도착점 표시 (zorder를 높여 선 위에 표시)
    plt.plot(start_pos_xy[0], start_pos_xy[1], 'bs', markersize=15, label='Start', zorder=2) 
    plt.plot(goal_pos_yx[1], goal_pos_yx[0], 'rs', markersize=15, label='Goal', zorder=2)

    plt.title("A* Algorithm Maze Solver")
    plt.legend()
    plt.grid(True, color='gray', linestyle='--', linewidth=0.5)
    plt.show()

if __name__ == "__main__":
    main()