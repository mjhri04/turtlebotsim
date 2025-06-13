from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
                # Velocity command (ROS2 -> IGN)
                '/cmd_vel_half@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                ],
        remappings=[
            ("/odom/tf", "tf"),
        ],
        output='screen'
    )

    map_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'])

    return LaunchDescription([
        bridge,
        map_static_tf,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
    ])
