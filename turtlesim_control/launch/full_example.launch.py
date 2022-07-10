import imp
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    turtle_following_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlesim_control'),
                'launch',
                'turtle_following.launch.py'
            ])
        ]),
        launch_arguments={
            'new_background_r' : '255',
        }.items()
    )

    leader = Node(
        package='turtlesim_control',
        executable='via_point_follower.py',
        parameters=[
            {'gain':5.0}
        ],
        remappings=[
            ('/cmd_vel','/turtle1/cmd_vel'),
            ('/pose', '/turtle1/pose'),
        ],

    )

    scheduler = Node(
        package='turtlesim_control',
        executable='scheduler.py',
    )

    entity_to_run = [scheduler,leader,turtle_following_launch]
    return LaunchDescription(entity_to_run)