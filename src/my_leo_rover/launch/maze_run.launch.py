from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 시뮬레이터 노드 실행
        Node(
            package='my_leo_rover',
            executable='maze_run',
            name='simulator'
        ),
        # 2. AI 두뇌 노드 실행
        Node(
            package='my_leo_rover',
            executable='maze_brain',
            name='brain'
        )
    ])
