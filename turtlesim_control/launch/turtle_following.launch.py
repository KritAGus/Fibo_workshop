#!/usr/bin/python3
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess,DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    new_background_r = LaunchConfiguration('new_background_r')
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='100'

    )
        
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        parameters=[
            {'background_r':new_background_r},
            {'background_g':255},
            {'background_b':0},
        ]
    )

    config = os.path.join(
        get_package_share_directory('turtlesim_control'),
        'config',
        'follower_config.yaml',
    )
    

    follower = Node(
        package='turtlesim_control',
        executable='turtle_follower.py',
        parameters=[
            config
        ],
        remappings=[
            ('/cmd_vel','/turtle2/cmd_vel'),
            ('/pose', '/turtle2/pose'),
            ('/goal', '/turtle1/pose')
        ],
    )

    spawn_turtle2 = ExecuteProcess(
        cmd = [['ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: \'turtle2\'}"']],
        shell = True
    )

    entity_to_run = [turtlesim,follower,spawn_turtle2,new_background_r_launch_arg]
    return LaunchDescription(entity_to_run)