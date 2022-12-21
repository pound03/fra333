#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import numpy as np
from enum import Enum

class commy(Enum):
    stop = 0
    rotationccw = 1
    rotationcw = 2
    forward = 3
    backward = 4
    up = 5
    down = 6
    left = 7 # [x,y,0] x [0,0,1]
    right = 8

class lab3_Control(Node):
    def __init__(self):
        super().__init__('lab3_Control')
        self.get_logger().info('lab3_Control has started')
        self.subArudino = self.create_subscription(Imu, '/Imu_arduino', self.arduino_callback, 10)
        self.pubToFilter = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.subFiltered = self.create_subscription(Imu, '/imu/data', self.filtered_callback, 10)
        # self.Timer = self.create_timer(0.1, self.timer_callback)

        self.ImuRawData = Imu()
        self.ImuFiltered = Imu()

        publish_topic = "/joint_trajectory_position_controller/joint_trajectory"
        self.trajectory_publihser = self.create_publisher(JointTrajectory,publish_topic, 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joints = ['joint_1','joint_2','joint_3']

        self.dh = np.array([[0,0,0.7,0],[-0.10,np.pi/2,-0.25,0],[1.00,0.0,0.35,0]]) #dh parameter
        self.q = np.array([0.0,0.0,0]) #joint angle,potision
        self.p = np.array([1,1,1]) #configuration of the robot
        self.h_endfactor = self.transx(1.15) #end factor transformation matrix
        self.pose = np.array([1.5,0.0,1.20]) #initial pose of the robot [x,y,z]
        self.armconfig = np.array([0,1]) #initial arm configuration
        self.Inverse_kinematics()

        self.circleCoordinates = np.array([0.0,0.0,0.0]) # [radius,angle,z]
        self.xyz2circleCoordinates()

        self.deltaTheta = 0.05
        self.deltaR = 0.005
        self.deltaZ = 0.02

        self.maxR = 1.5
        self.minR = 1.0
        self.maxZ = 1.7
        self.minZ = 0.5
        self.maxTheta = np.pi
        self.minTheta = -np.pi/2


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
            self.get_logger().info('No solution')
            return 0
        else:
            self.q[0] = q1 #เก็บค่า joint angle 1 ลงในตัวแปร q
            self.q[1] = q2 #เก็บค่า joint angle 2 ลงในตัวแปร q
            self.q[2] = q3 #เก็บค่า joint angle 3 ลงในตัวแปร q
            return 1

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

    def circleCoordinates2xyz(self):
        self.pose[0] = self.circleCoordinates[0]*np.cos(self.circleCoordinates[1])
        self.pose[1] = self.circleCoordinates[0]*np.sin(self.circleCoordinates[1])
        self.pose[2] = self.circleCoordinates[2]

    def xyz2circleCoordinates(self):
        self.circleCoordinates[0] = np.sqrt(self.pose[0]**2+self.pose[1]**2)
        self.circleCoordinates[1] = np.arctan2(self.pose[1],self.pose[0])
        self.circleCoordinates[2] = self.pose[2]

    def arduino_callback(self, msg):
        self.ImuRawData = msg
        self.pubToFilter.publish(msg)

    def filtered_callback(self, msg):
        threshold = 0.3
        thresholdAcc = 8.0
        self.ImuFiltered= msg
        self.command = commy.stop

        if(self.ImuFiltered.linear_acceleration.x > thresholdAcc):
            self.command = commy.backward
            # self.get_logger().info('Command: backward')
        elif(self.ImuFiltered.linear_acceleration.x < -thresholdAcc):
            self.command = commy.forward
            # self.get_logger().info('Command: forward')
        elif self.ImuFiltered.angular_velocity.y > threshold:
            self.command = commy.down
            # self.get_logger().info('Command: down')
        elif self.ImuFiltered.angular_velocity.y < -threshold:
            self.command = commy.up
            # self.get_logger().info('Command: up')
        elif self.ImuFiltered.angular_velocity.z > threshold:
            self.command = commy.rotationccw
            # self.get_logger().info('Command: rotationccw')
        elif self.ImuFiltered.angular_velocity.z < -threshold:
            self.command = commy.rotationcw
            # self.get_logger().info('Command: rotationcw')
        self.control_main()

    def control_main(self):
        if(self.command == commy.stop):
            pass
        elif(self.command == commy.forward):
            self.circleCoordinates[0] += self.deltaR
        elif(self.command == commy.backward):
            self.circleCoordinates[0] -= self.deltaR
        elif(self.command == commy.up):
            self.circleCoordinates[2] += self.deltaZ*1.2
        elif(self.command == commy.down):
            self.circleCoordinates[2] -= self.deltaZ
        elif(self.command == commy.rotationcw):
            self.circleCoordinates[1] -= self.deltaTheta
        elif(self.command == commy.rotationccw):
            self.circleCoordinates[1] += self.deltaTheta

        if(self.circleCoordinates[0] > self.maxR):
            self.circleCoordinates[0] = self.maxR
        elif(self.circleCoordinates[0] < self.minR):
            self.circleCoordinates[0] = self.minR
        if(self.circleCoordinates[2] > self.maxZ):
            self.circleCoordinates[2] = self.maxZ
        elif(self.circleCoordinates[2] < self.minZ):
            self.circleCoordinates[2] = self.minZ
        if(self.circleCoordinates[1] > self.maxTheta):
            self.circleCoordinates[1] = self.circleCoordinates[1] - 2*np.pi
        elif(self.circleCoordinates[1] < self.minTheta):
            self.circleCoordinates[1] = self.circleCoordinates[1] + 2*np.pi
        print(self.circleCoordinates)
        self.circleCoordinates2xyz()
        self.Inverse_kinematics()

        self.command = commy.stop


    def timer_callback(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joints
        ## creating a point
        point = JointTrajectoryPoint()
        point.positions = self.q.tolist()
        nanasec = int(0.5 * 1e9)
        point.time_from_start = Duration(sec=0, nanosec=nanasec)
        ## adding newly created point into trajectory message
        trajectory_msg.points.append(point)
        # point.positions = self.goal_positions
        # point.time_from_start = Duration(sec=8)
        # bazu_trajectory_msg.points.append(point)
        self.trajectory_publihser.publish(trajectory_msg)



def main(args=None):
    rclpy.init(args=args)
    node = lab3_Control()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
