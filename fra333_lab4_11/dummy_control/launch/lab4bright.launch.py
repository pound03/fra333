#!usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import sys
import os
def generate_launch_description():
    launch_description = LaunchDescription()

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
    gazebo = IncludeLaunchDescription(
         PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dummy_description'),
                'launch',
                'dummy_gazebo.launch.py'
            ])
        ])
    )


    config = PathJoinSubstitution([
        FindPackageShare('dummy_control'),
        'config',
        'tracker_config.yaml'
    ])
    controller = Node(
        package='dummy_control',
        executable='tracker.py',
        parameters=[config]
    )

    launch_description.add_action(gazebo)
    launch_description.add_action(controller)
    launch_description.add_action(rviz)
    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()

if __name__ == '__main__':
    main()
    