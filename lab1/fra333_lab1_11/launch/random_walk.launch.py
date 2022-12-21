#!usr/bin/python3
import turtle
from launch import LaunchDescription
from launch.actions import ExecuteProcess,DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    launch_description = LaunchDescription() # ประกาศ object ของ LaunchDescription เพื่อที่จะเก็บค่าที่ node / cmd ต่างๆ ไปให้ launch file

    turtle = Node( 
        package='turtlesim',
        executable='turtlesim_node',
    )# ประกาศ object turtle ของ Node เพื่อที่ไว้รัน turtle node
    launch_description.add_action(turtle)

    rate = LaunchConfiguration('rate') # ประกาศ object rate ของ LaunchConfiguration เพื่อที่จะเก็บ point ที่เราใส่ใน สามารถเรียกใช้ได้โดยใช้ object rate แทน
    rate_launch_arg = DeclareLaunchArgument('rate',default_value='1.0') # ประกาศ object rate_launch_arg ของ DeclareLaunchArgument เพื่อที่จะเก็บค่าที่ argument ที่รับเข้ามาจาก cmd และกำหนดค่า default ให้กับ argument นั้นๆ
    launch_description.add_action(rate_launch_arg) # เพิ่มค่าที่เก็บไว้ใน object rate_launch_arg ให้กับ launch_description

    linear = Node( # ประกาศ object linear ของ Node เพื่อที่จะเก็บค่าที่ node ต่างๆ ไปให้ launch file
        package='fra333_lab1_11', # ชื่อ package ที่เราต้องการใช้
        executable='noise_generator.py', # ชื่อ node ที่เราต้องการใช้
        namespace= 'linear', # กำหนด namespace
        arguments=[rate], # กำหนด argument เข้าไปใน node โดยใช้ object rate ที่เราประกาศไว้
    )
    launch_description.add_action(linear) # เพิ่มค่าที่เก็บไว้ใน object linear ให้กับ launch_description

    angular = Node( # ประกาศ object angular ของ Node เพื่อที่จะเก็บค่าที่ node ต่างๆ ไปให้ launch file
        package='fra333_lab1_11', # ชื่อ package ที่เราต้องการใช้
        executable='noise_generator.py', # ชื่อ node ที่เราต้องการใช้
        namespace= 'angular', # กำหนด namespace เราใช้คนละ namespace กับ linear เพื่อให้ไม่เกิดการเขียนทับกัน
        arguments=[rate], # กำหนด argument เข้าไปใน node โดยใช้ object rate ที่เราประกาศไว้
    )
    launch_description.add_action(angular) # เพิ่มค่าที่เก็บไว้ใน object angular ให้กับ launch_description


    controller = Node( # ประกาศ object controller ของ Node เพื่อที่จะเก็บค่าที่ node ต่างๆ ไปให้ launch file
        package='fra333_lab1_11', # ชื่อ package ที่เราต้องการใช้
        executable='velocity_mux.py', # ชื่อ node ที่เราต้องการใช้
        arguments=[rate], # กำหนด argument เข้าไปใน node โดยใช้ object rate ที่เราประกาศไว้
    )
    launch_description.add_action(controller) # เพิ่มค่าที่เก็บไว้ใน object controller ให้กับ launch_description


    mean_linear = 1.0 # กำหนดค่า mean ของ linear ให้เป็น 1.0
    variance_linear = 0.0 # กำหนดค่า variance ของ linear ให้เป็น 0.0 
    call_service_linear = ExecuteProcess( 
        cmd=['ros2','service','call','/linear/set_noise','lab1_interfaces/srv/SetNoise','"{mean: ',str(mean_linear),', variance: ',str(variance_linear),'}"'],
        shell=True,
    ) # กำหนดค่าให้กับ object call_service_linear โดยใช้คำสั่ง ros2 service call ในการเรียกใช้ service ที่ชื่อ linear/set_noise โดยใช้ package lab1_interfaces และ service type ที่ชื่อ SetNoise โดยใช้ค่า mean และ variance ที่เรากำหนดไว้
    launch_description.add_action(call_service_linear) # เพิ่มค่าที่เก็บไว้ใน object call_service_linear ให้กับ launch_description
    
    
    mean_angular = 0.0 # กำหนดค่า mean ของ angular ให้เป็น 0.0
    variance_anugular = 3.0 # กำหนดค่า variance ของ angular ให้เป็น 3.0
    call_service_anguler = ExecuteProcess(
        cmd=['ros2','service','call','/angular/set_noise','lab1_interfaces/srv/SetNoise','"{mean: ',str(mean_angular),', variance: ',str(variance_anugular),'}"'],
        shell=True,
    ) # กำหนดค่าให้กับ object call_service_anguler โดยใช้คำสั่ง ros2 service call ในการเรียกใช้ service ที่ชื่อ angular/set_noise โดยใช้ package lab1_interfaces และ service type ที่ชื่อ SetNoise โดยใช้ค่า mean และ variance ที่เรากำหนดไว้
    launch_description.add_action(call_service_anguler) # เพิ่มค่าที่เก็บไว้ใน object call_service_anguler ให้กับ launch_description

    return launch_description # ส่งค่า launch_description กลับไป


if __name__=='__main__':
    generate_launch_description()
