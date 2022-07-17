#!/usr/bin/env python3
import os
from time import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import sys

#sudo apt install ros-foxy-joint-state-publisher
#sudo apt install ros-foxy-joint-state-publisher-gui
def generate_launch_description():
    # Get the launch directory
    arobota_dir = get_package_share_directory('arobota_descriptions')
    rviz_file_name = 'arobota_config.rviz'
    rviz_file_path = os.path.join(arobota_dir, 'rviz', rviz_file_name)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_share = get_package_share_directory('arobota_descriptions')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'arobota.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    params = {'robot_description': robot_desc}

    rviz_Node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_file_path],
        output='screen')

    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=[params]
    )

    joint_state_publisher = Node(package='joint_state_publisher',
                                    executable='joint_state_publisher',
                                    name='joint_state_publisher'
    )
    joint_state_publisher_gui = Node(package='joint_state_publisher_gui',
                                        executable='joint_state_publisher_gui',
                                        name='joint_state_publisher_gui'
    )

    ld = LaunchDescription()
    ld.add_action(rviz_Node)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(joint_state_publisher_gui)

    return ld

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()

if __name__ == '__main__':
    main()
    