from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    turtlesim_description_dir = get_package_share_directory('turtlesim_descriptions')
    rviz_file_name = 'turtlesim_tf2_broadcaster_rviz.rviz'
    rviz_file_path = os.path.join(turtlesim_description_dir,'rviz',rviz_file_name)
   
    full_example_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
               FindPackageShare('turtlesim_control'),
               'launch',
               'full_example.launch.py',
            ]),
        ]),
    )

    tf2_turtle1 = Node(
        package="turtlesim_descriptions",
        executable='tf2_broadcaster.py',
        arguments=['turtle1']
    )
   
    tf2_turtle2 = Node(
        package="turtlesim_descriptions",
        executable='tf2_broadcaster.py',
        arguments=['turtle2']
    )

    rviz = Node(
        package="rviz2",
        executable='rviz2',
        arguments=['-d', rviz_file_path],
        output='screen'
    )

    entity_to_run = [full_example_launch,tf2_turtle1,tf2_turtle2,rviz]
    return LaunchDescription(entity_to_run)