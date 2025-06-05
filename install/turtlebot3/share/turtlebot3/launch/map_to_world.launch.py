from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3',  # 너의 패키지 이름
            executable='map_to_world_node',
            name='map_to_world_node',
            output='screen'
        )
    ])
