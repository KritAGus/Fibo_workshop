#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        parameters=[
            {'background_r':10},
            {'background_g':100},
            {'background_b':50},
        ]
    )

    controller = Node(
        package='turtlesim_control',
        executable='controller.py',
        parameters=[
            {'gain':10.0}
        ],
        remappings=[
            ('/turtle1/cmd_vel','/turtle2/cmd_vel'),
            ('turtle1/pose', '/turtle2/pose'),
        ],
    )

    scheduler = Node(
        package='turtlesim_control',
        executable='scheduler.py',
    )

    spawn_turtle2 = ExecuteProcess(
        cmd = [['ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: \'turtle2\'}"']],
        shell = True
    )

    entity_to_run = [turtlesim,controller,scheduler,spawn_turtle2]
    return LaunchDescription(entity_to_run)