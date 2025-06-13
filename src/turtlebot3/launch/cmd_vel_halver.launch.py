from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3',     
            executable='cmd_vel_halver',
            remappings=[
                ('/odom', '/sim/odom'),
            ],
            name='cmd_vel_halver',
            output='screen'
        )
    ])
