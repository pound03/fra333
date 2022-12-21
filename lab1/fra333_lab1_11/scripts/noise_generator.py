#!/usr/bin/python3

# import all other neccesary libraries here

import sys
import rclpy
from rclpy.node import Node
import numpy as np
from lab1_interfaces.srv import SetNoise
from std_msgs.msg import Float64
class NoiseGenerator(Node): # ประกาศ class ชื่อ NoiseGenerator ที่สืบทอดมาจาก class Node

    def __init__(self):
        #check have argument or not
        if len(sys.argv)>2: #check len of sys.argv
            # have argument set rate to argument value
            self.rate = float(sys.argv[1])
        else:
            # no argument set rate to 5
            self.rate = 5.0
        # init node สืบทอด method จาก class Node
        super().__init__('noise_generator') 
        # สร้างตัวแปร set_noise_service ที่เป็น server service ชื่อ set_noise และ callback เป็น self.set_noise_callback กำหนดให้ชนิดข้อมูลเป็น SetNoise
        # set_noise_service เป็น object class Service ที่สร้างขึ้นมาจากการเรียกใช้ funtion create_service ที่สืมทอดมาจาก class Node
        self.set_noise_service = self.create_service(SetNoise,'set_noise',self.set_noise_callback) 
        # กำหนดค่า timer_period โดย 1/rate rate คือ ค่าที่รับมาจาก argument (hz) หรือ 5 ถ้าไม่มี argument
        timer_period = 1.0/self.rate
        # สร้าง timer โดยใช้ค่า timer_period และ มี callback เป็น funtion self.timer_callback
        # timer เป็น object class Timer ที่สร้างขึ้นมาจากการเรียกใช้ funtion create_timer ที่สืมทอดมาจาก class Node
        self.timer = self.create_timer(timer_period,self.timer_callback) 
        # สร้าง publisher ชื่อ noise และ กำหนดให้เป็น Float64 และ ไว้สำหรับส่งค่า signal
        # command_publisher เป็น object ของ publisher ที่สร้างขึ้นมา จากการเรียก funtion create_publisher ที่สืมทอดมาจาก class Node
        self.command_publisher = self.create_publisher(Float64,'noise',10) 
        # กำหนดค่า mean เริ่มต้นเป็น 0.0
        self.mean = 0.0 
        # กำหนดค่า variance เริ่มต้นเป็น 1.0
        self.variance = 1.0 

    def set_noise_callback(self,request:SetNoise.Request,response:SetNoise.Response):
        # funtion ที่ทำงานเมื่อมีการเรียกใช้ service ชื่อ set_noise โดยรับค่าจาก request ซึ่งมี data ชื่อ mean และ variance
        self.mean = request.mean # กำหนดค่า mean ให้เท่ากับค่าที่รับมาจาก request
        self.variance = request.variance # กำหนดค่า variance ให้เท่ากับค่าที่รับมาจาก request

        return response # ส่งค่า response กลับไป เปป็นค่า empty
    
    def timer_callback(self):
        # funtion ที่ทำงานเมื่อ timer เกิดการ tick โดยจะส่งค่าที่ random_gussian ใน range ของ mean และ variance ที่รับมาจาก request ของ service 
        # และทำการ publisher ข้อมูล ไปยัง topic noise โดยใช้ object ชื่อ command_publisher ที่สร้างไว้ใน funtion __init__
        self.command_publisher.publish(Float64(data=np.random.normal(self.mean,self.variance)))
        pass

def main(args=None):
    rclpy.init(args=args) # สั่งให้ rclpy ทำการ init โดยใช้ค่า args ที่รับมาจาก main
    controller = NoiseGenerator() # สร้างตัวแปร controller ที่เป็น object ของ class NoiseGenerator
    rclpy.spin(controller) # สั่งให้ rclpy ทำการ spin โดยใช้ค่า controller ที่สร้างไว้ในตัวแปร controller
    controller.destroy_node() # สั่งให้ controller ทำการ destroy_node เมื่อ rclpy ทำการ spin จบ
    rclpy.shutdown() # สั่งให้ rclpy ทำการ shutdown
    pass

if __name__=='__main__':
    main()
