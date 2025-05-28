import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']   # waffle

def generate_launch_description():
    # urdf_path = '/home/ju/turtlebotsim/src/turtlebot3_robotis/turtlebot3_description/urdf/turtlebot3_burger.urdf'
    sdf_path = '/home/ju/turtlebotsim/src/turtlebot3_robotis/turtlebot3_description/urdf/turtlebot3_burger.sdf'

    # with open(sdf_path, 'w') as sdf_file:
    #     sdf_output = subprocess.run(['gz', 'sdf', '-p', urdf_path], stdout=subprocess.PIPE, check=True)
    #     sdf_file.write(sdf_output.stdout.decode())

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='turtlebot3_world')

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3'), 'launch')

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',value=[
        os.path.join("/opt/ros/humble", "share"),
        ":" +
        os.path.join(get_package_share_directory('turtlebot3'), "models")])

    # Spawn robot
    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-entity', TURTLEBOT3_MODEL,
                   '-name', TURTLEBOT3_MODEL,
                   '-file', sdf_path,
                   '-allow_renaming', 'true',
                   '-x', '-2.0',
                   '-y', '-0.5',
                   '-z', '0.01'],
        )
    
    # Spawn world
    ignition_spawn_world = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-file', PathJoinSubstitution([
                        get_package_share_directory('turtlebot3'),
                        "models", "worlds", "model.sdf"]),
                   '-allow_renaming', 'false'],
        )
    
    world_only = os.path.join(get_package_share_directory('turtlebot3'), "models", "worlds", "world_only.sdf")

    return LaunchDescription([
        ign_resource_path,
        ignition_spawn_entity,
        ignition_spawn_world,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
            launch_arguments=[('ign_args', [' -r -v 3 ' +
                              world_only
                             ])]),
                             
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),

        DeclareLaunchArgument(
            'world_name',
            default_value=world_name,
            description='World name'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/ros_ign_bridge.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/navigation2.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
