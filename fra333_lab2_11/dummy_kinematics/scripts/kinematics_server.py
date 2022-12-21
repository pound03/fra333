#!/usr/bin/python3

from math import gamma
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from dummy_kinematics_interfaces.srv import SetFK
from dummy_kinematics_interfaces.srv import SolveIK
#import float32array
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
#import float64 array msg

class forward_kinematics(Node):
    def __init__(self):
        super().__init__('dummy_node')
        self.publisher_joint_state = self.create_publisher(JointState, 'joint_states', 10) #สร้าง topic ชื่อ joint_states สำหรับส่งข้อมูลออกไปที่ joint_state_publisher
        self.timer = self.create_timer(0.1, self.control_joint) #สร้าง timer ที่ทำงานทุกๆ 0.1 วินาที ที่จะเรียกใช้งานฟังก์ชัน control_joint สำหรับส่งข้อมูลออกไปที่ joint_state_publisher
        self.server_set_joint = self.create_service(SetFK, 'set_joint', self.set_joint_callback) #สร้าง service ชื่อ set_joint สำหรับรับข้อมูลจาก client ไว้สำหรับกำหนดค่า joint ของ robot
        self.server_solve_ik = self.create_service(SolveIK, 'solve_ik', self.solve_ik_callback) #สร้าง service ชื่อ solve_ik สำหรับรับข้อมูลจาก client ไว้สำหรับคำนวณค่า joint ของ robot จากข้อมูล pose ของ end-effector
        self.dh = np.array([[0,0,0.7,0],[-0.10,np.pi/2,-0.25,0],[1.00,0.0,0.35,0]]) #dh parameter
        self.q = np.array([0.0,0,0]) #joint angle,potision
        self.p = np.array([1,1,1]) #configuration of the robot
        self.h_endfactor = self.transx(1.15) #end factor transformation matrix
        self.pose = np.array([0.0,0.0,0.0]) #initial pose of the robot
        self.armconfig = np.array([0,0]) #initial arm configuration

    def control_joint(self): 
        msg = JointState() #สร้างข้อมูลของ joint_state
        msg.header.stamp = self.get_clock().now().to_msg() # กำหนด timestamp ของข้อมูล
        msg.name = ['joint_1','joint_2','joint_3'] #กำหนดชื่อของ joint
        msg.position = [float(self.q[0]),float(self.q[1]),float(self.q[2])] #กำหนดค่าของ joint
        self.publisher_joint_state.publish(msg) #ส่งข้อมูลออกไปที่ joint_state_publisher

    def rotx(self, theta): #rotation matrix around x-axis
        return np.array([[1,0,0,0],[0,np.cos(theta),-np.sin(theta),0],[0,np.sin(theta),np.cos(theta),0],[0,0,0,1]])
    def rotz(self, theta): #rotation matrix around z-axis
        return np.array([[np.cos(theta),-np.sin(theta),0,0],[np.sin(theta),np.cos(theta),0,0],[0,0,1,0],[0,0,0,1]])
    def transx(self, d): #translation matrix along x-axis
        return np.array([[1.0,0,0,d],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    def transz(self, d): #translation matrix along z-axis
        return np.array([[1.0,0,0,0],[0,1,0,0],[0,0,1,d],[0,0,0,1]])
    
    def RRIK(self,x,y,gamma,l1,l2): #ไว้สำหรับ solve inverse kinematics รูปแบบ revolute-revolute (RR)
        c2 = (x**2+y**2-l1**2-l2**2)/(2*l1*l2) #cosine of joint angle 2
        s2 = np.sqrt(1-c2**2)*gamma #sine of joint angle 2
        q2 = np.arctan2(s2,c2) #joint angle 2
        q1 = np.arctan2(y,x)-np.arctan2(l2*s2,l1+l2*c2) #joint angle 1
        return q1,q2 #return joint angle 1 and joint angle 2
    def Inverse_kinematics(self): #ไว้สำหรับ solve inverse kinematics ของ robot
        x = self.pose[0] #x position of the end-effector
        y = self.pose[1] #y position of the end-effector
        z = self.pose[2] #z position of the end-effector
        h = 0.7 #height of the robot
        l1 = 0.10 #length of link 1
        l2 = 1.00 #length of link 2
        l3 = 1.15 #length of link 3
        a = -0.1 #offset in x axis of end-effector
        if(self.armconfig[0] == 0): #set gamma = 1
            gamma1 = 1
        else:
            gamma1 = -1
        if(self.armconfig[1] == 0):
            gamma2 = 1
        else:
            gamma2 = -1
        
        r = np.sqrt(x**2+y**2-a**2)*gamma1 # scalar of the position of the end-effector in the x-y plane
        det = r**2+a**2 #determinant of the position of the end-effector in the x-y plane
        s1 = -a/det*x + r/det*y #sine of joint angle 1
        c1 = r/det*x + a/det*y #cosine of joint angle 1
        q1 = np.arctan2(s1,c1) #joint angle 1
        q2,q3 = self.RRIK(r+l1,z-h,gamma2,l2,l3) #joint angle 2 and joint angle 3 solve by RRIK function
        if(q1 == np.nan or q2 == np.nan or q3 == np.nan):
            print("No solution")
        else:
            self.q[0] = q1 #เก็บค่า joint angle 1 ลงในตัวแปร q
            self.q[1] = q2 #เก็บค่า joint angle 2 ลงในตัวแปร q
            self.q[2] = q3 #เก็บค่า joint angle 3 ลงในตัวแปร q

    def solve_ik_callback(self, request, response):
        # ฟังก์ชันสำหรับรับค่าจาก service และคำนวณ inverse kinematics
        # print(request)
        self.pose[0] = request.position.x
        self.pose[1] = request.position.y
        self.pose[2] = request.position.z
        self.armconfig = request.arm_config
        self.Inverse_kinematics()
        # response.joint_angles = [self.q[0]*180/np.pi*180/np.pi,self.q[1]*180/np.pi,self.q[2]*180/np.pi] #แปลงค่า joint angle ให้เป็นองศา
        response.joint_angles = [self.q[0],self.q[1],self.q[2]] #แปลงค่า joint angle ให้เป็นองศา
        return response

    def forward_kinematics(self):
        h = np.array([[1.0,0.0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]) #initial identity matrix 4x4 for homogeneous transformation
        for i in range(0,3): #loop for each joint
            tx = self.transx(self.dh[i,0]) #translation matrix along x-axis
            rx = self.rotx(self.dh[i,1]) #rotation matrix around x-axis
            tz = self.transz(self.dh[i,2]) #translation matrix along z-axis
            rz = self.rotz(self.dh[i,3]) #rotation matrix around z-axis
            if(self.p[i] == 1): #if joint is revolute
                H_j = self.rotz(self.q[i]) #rotation matrix around z-axis
                H_j = np.round_(H_j,3) #round the value of the matrix to 3 decimal places
            else :
                H_j = self.transz(self.q[i]) #translation matrix along z-axis
            H_mut = tx@rx@tz@rz #homogeneous transformation matrix from dh parameter
            H_mut = np.round(H_mut,3) #round the value of the matrix to 3 decimal places
            # print(H_mut , 'H_mut')
            # print(H_j , 'H_j')

            h = h@H_mut #calculate h with homogeneous transformation matrix from dh parameter
            h = h@H_j #calculate h with homogeneous transformation matrix from input joint angle
            h = np.round_(h,3) #round the value of the matrix to 3 decimal places
            
            # print(h,f'frame : {i}')
        h = h@self.h_endfactor #calculate h with homogeneous transformation matrix from end-effector
        h = np.round_(h,3) #round the value of the matrix to 3 decimal places
        # print(h)
        self.pose[0] = h[0,3] #x position of the end-effector
        self.pose[1] = h[1,3] #y position of the end-effector
        self.pose[2] = h[2,3] #z position of the end-effector
        # print(self.pose)
    def set_joint_callback(self, request, response):
        # ฟังก์ชันสำหรับรับค่าจาก service ในหน่วยองศา และคำนวณ forward kinematics
        # print(request)
        self.q[0] = request.joint_state.position[0]
        self.q[1] = request.joint_state.position[1]
        self.q[2] = request.joint_state.position[2]
        # print(self.q)
        # self.q[0] = self.q[0]*np.pi/180
        # self.q[1] = self.q[1]*np.pi/180
        # self.q[2] = self.q[2]*np.pi/180
        #เปลี่ยนค่า joint angle ให้เป็น radian
        self.forward_kinematics()
        msg = Point()
        msg.x = self.pose[0]
        msg.y = self.pose[1]
        msg.z = self.pose[2]
        response.position = msg
        return response

def main(args=None):
    rclpy.init(args=args)
    node = forward_kinematics()
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown() 

if __name__=='__main__':
    main()
