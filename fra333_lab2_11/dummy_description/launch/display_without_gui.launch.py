#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import sys

def generate_launch_description():

    path_to_package = get_package_share_directory("dummy_description")
    sub_folder = 'config'
    file_name = 'dummy_kinematics.rviz'
    rviz_file_path = os.path.join(path_to_package,sub_folder,file_name) 
    
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       name='rviz',
       arguments=['-d', rviz_file_path],
       output='screen')
    
    ### How to read URDF file ###
    #

    path_to_package = get_package_share_directory("dummy_description")
    sub_folder = 'robot'
    file_name = 'dummy.urdf'
    robot_description_path = os.path.join(path_to_package,sub_folder,file_name) 
    
    with open(robot_description_path, 'r') as infp:
        robot_description = infp.read()

    joint_state_publisher_gui = Node(package='joint_state_publisher_gui',
                                        executable='joint_state_publisher_gui',
                                        name='joint_state_publisher_gui'
    )

    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=[
                                    {'use_sim_time': False},
                                    {'robot_description': robot_description}
                                  ]
    )

    launch_description = LaunchDescription()
    launch_description.add_action(rviz)
    launch_description.add_action(robot_state_publisher)
    # launch_description.add_action(joint_state_publisher_gui)
    
    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()

if __name__ == '__main__':
    main()
    