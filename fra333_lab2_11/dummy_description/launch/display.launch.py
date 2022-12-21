#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import sys
from launch.actions import ExecuteProcess

def generate_launch_description():

    path_to_package = get_package_share_directory("dummy_description") # get path to package
    sub_folder = 'config' # set subfolder name
    file_name = 'dummy_kinematics.rviz' # set file name
    rviz_file_path = os.path.join(path_to_package,sub_folder,file_name) # get full path to file setting in rviz
    
    rviz = Node( # run rviz2
       package='rviz2',
       executable='rviz2',
       name='rviz',
       arguments=['-d', rviz_file_path], # load setting
       output='screen')

    path_to_package = get_package_share_directory("dummy_description") 
    sub_folder = 'robot'
    file_name = 'dummy.urdf'
    robot_description_path = os.path.join(path_to_package,sub_folder,file_name) # get full path to urdf file
    
    with open(robot_description_path, 'r') as infp: #read urdf file
        robot_description = infp.read()
    
    robot_state_publisher = Node(package='robot_state_publisher', # run robot_state_publisher
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=[
                                    {'use_sim_time': False},
                                    {'robot_description': robot_description} # load urdf file to robot_state_publisher 
                                  ]
    )
    joint_state_publisher_gui = Node(package='joint_state_publisher_gui', # run joint_state_publisher_gui เพื่อให้สามารถกำหนดค่า joint ได้
                                    executable='joint_state_publisher_gui',
                                    name='joint_state_publisher_gui'
    )

    launch_description = LaunchDescription()
    launch_description.add_action(rviz)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(joint_state_publisher_gui)
    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()

if __name__ == '__main__':
    main()
    