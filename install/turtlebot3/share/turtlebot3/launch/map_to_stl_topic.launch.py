from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3',
            executable='map_to_stl_topic',
            name='map_to_stl_topic',
            output='screen'
        )
    ])
