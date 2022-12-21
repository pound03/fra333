#!usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import sys

def generate_launch_description():
    launch_description = LaunchDescription()

    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dummy_description'),
                'launch',
                'display.launch.py'
            ])
        ])
    )
    gazebo = IncludeLaunchDescription(
         PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dummy_description'),
                'launch',
                'dummy_gazebo.launch.py'
            ])
        ])
    )

    launch_description.add_action(rviz2)
    launch_description.add_action(gazebo)
    # launch_description.add_action(controller)
    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()

if __name__ == '__main__':
    main()
    