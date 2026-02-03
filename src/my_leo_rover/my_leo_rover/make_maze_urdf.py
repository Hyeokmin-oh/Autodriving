import os
import random

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
    urdf_content = '<?xml version="1.0" ?>\n<robot name="maze">\n'
    
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
    <inertial>
      <mass value="0"/>
      <origin xyz="{x} {y} 0.5"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
  </link>
  <joint name="joint_{wall_count}" type="fixed">
    <parent link="base_link"/>
    <child link="wall_{wall_count}"/>
  </joint>
"""
    # Base link 추가
    base_link = '  <link name="base_link"/>\n'
    urdf_content = urdf_content.replace('<robot name="maze">', '<robot name="maze">\n' + base_link)
    urdf_content += "</robot>"

    with open(filename, "w") as f:
        f.write(urdf_content)
    print(f"성공: {filename} 파일이 생성되었습니다. (벽 개수: {wall_count})")

if __name__ == "__main__":
    # 파일 저장 경로 설정 (사용자 폴더 구조 반영)
    save_path = "/root/ros2_ws/src/my_leo_rover/my_leo_rover/maze.urdf"
    my_maze = create_maze(15, 15)
    save_to_urdf(my_maze, save_path)