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

    lab3_control = Node(
        package='lab3_control',
        executable='lab3_Control.py',)

    xicro = Node(
        package='xicro_pkg',
        executable='xicro_node_read_imu_ID_3_arduino.py',
        arguments=['/dev/ttyACM0'],
        )
    imu_complementary_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('imu_complementary_filter'),
                'launch',
                'complementary_filter.launch.py'
            ])
        ])
    )
    spawn_robot = IncludeLaunchDescription(
         PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lab3_control'),
                'launch',
                'dummy_gazebo.launch.py'
            ])
        ])
    )

    launch_description.add_action(lab3_control)
    launch_description.add_action(xicro)
    launch_description.add_action(imu_complementary_filter)
    launch_description.add_action(spawn_robot)
    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()

if __name__ == '__main__':
    main()
    