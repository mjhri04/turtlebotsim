import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3',  # 당신의 패키지 이름
            executable='goal_pose_to_nav2',
            name='goal_pose_to_nav2',
            output='screen'
        )
    ])
