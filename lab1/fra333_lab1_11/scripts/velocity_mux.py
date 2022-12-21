#!/usr/bin/python3

# import all other neccesary libraries
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.node import Node
import rclpy
import sys

class VelocityMux(Node):
    def __init__(self):
        # เช็คว่ามี argument ใส่เข้ามาหรือไม่
        if len(sys.argv)>2:
            # ถ้ามีให้ใช้ argument ที่ใส่เข้ามา แทนที่ค่า ในตัวแปร self.rate
            self.rate = float(sys.argv[1])
        else:
            # ถ้าไม่มีให้ใช้ค่า 5.0 แทน
            self.rate = 5.0
        # สร้าง node ชื่อ velocity_mux
        super().__init__('velocity_mux')

        # สร้าง publisher ชื่อ command โดยใช้ message type ของ Twist
        self.timer = self.create_timer(1.0/self.rate,self.timer_callback)
        
        self.cmd_vel = Twist() # ประกาศตัวแปร cmd_vel ขึ้นมาเพื่อเก็บค่า velocity ที่จะส่งไปให้ turtle
        self.get_logger().info(f'Starting {self.get_name()}') # แสดงข้อความใน terminal ว่า node นี้เริ่มทำงานแล้ว
        # สร้าง subscription ชื่อ linear/noise โดยใช้ message type ของ Float64 และใช้ callback function ชื่อ linear_vel_sub_callback
        # linear_subscription เป็น object ของ Subscription ที่สร้างขึ้นมา จากการเรียก funtion create_subscription ที่สืมทอดมาจาก class Node
        self.linear_subscription = self.create_subscription(Float64,'linear/noise',self.linear_vel_sub_callback,10)
        # สร้าง subscription ชื่อ angular/noise โดยใช้ message type ของ Float64 และใช้ callback function ชื่อ angular_vel_sub_callback
        # linear_subscription เป็น object ของ Subscription ที่สร้างขึ้นมา จากการเรียก funtion create_subscription ที่สืมทอดมาจาก class Node
        self.angular_subscription = self.create_subscription(Float64,'angular/noise',self.angular_vel_sub_callback,10)
        # สร้าง publisher ชื่อ command_publisher โดยใช้ message type ของ Twist
        # สำหรับส่งค่า velocity ไปให้ turtlesim
        self.command_publisher = self.create_publisher(Twist,'turtle1/cmd_vel',10)

        # สร้าง ตัวแปร สำหรับเก็บค่า linear velocity และ angular velocity
        self.l=0.0
        self.a=0.0
    def linear_vel_sub_callback(self,msg:Float64):
        # function นี้จะถูกเรียกเมื่อมี message ถูกส่งมาที่ topic ชื่อ linear/noise
        # เก็บค่า linear velocity ที่ส่งมาในตัวแปร self.l
        self.l=msg.data
        pass
    
    def angular_vel_sub_callback(self,msg:Float64):
        # function นี้จะถูกเรียกเมื่อมี message ถูกส่งมาที่ topic ชื่อ angular/noise
        # เก็บค่า angular velocity ที่ส่งมาในตัวแปร self.a
        self.a=msg.data
        pass
    
    def timer_callback(self):
        # funtion ที่ทำงานเมื่อ timer เกิดการ tick โดยจะส่งค่าที่ self.l และ self.a ไปให้ turtlesim 
        # และทำการ publisher ข้อมูล ไปยัง topic turtle1/cmd_vel โดยใช้ object ชื่อ command_publisher ที่สร้างไว้ใน funtion __init__
        msg=Twist() # บอกว่าจะส่ง message ชนิด Twist
        msg.linear.x=self.l # กำหนดค่า linear velocity ให้กับ msg.linear.x
        msg.angular.z=self.a # กำหนดค่า angular velocity ให้กับ msg.angular.z
        self.command_publisher.publish(msg) # ส่ง message ไปยัง topic turtle1/cmd_vel
        pass

def main(args=None):

    rclpy.init(args=args) # สั่งให้ rclpy ทำการ init โดยใช้ค่า args ที่รับมาจาก main
    controller = VelocityMux() # สร้างตัวแปร controller ที่เป็น object ของ class NoiseGenerator
    rclpy.spin(controller) # สั่งให้ rclpy ทำการ spin โดยใช้ค่า controller ที่สร้างไว้ในตัวแปร controller
    controller.destroy_node() # สั่งให้ controller ทำการ destroy_node เมื่อ rclpy ทำการ spin จบ
    rclpy.shutdown() # สั่งให้ rclpy ทำการ shutdown
    pass

if __name__=='__main__':
    main()