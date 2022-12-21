#!/usr/bin/python3

from dummy_control.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
class DummyNode(Node):
    def __init__(self):
        super().__init__('tracker')
        self.declare_parameters(namespace='', parameters=[('kp',None),('ki',None),('kd',None)])
        self.kp = self.get_parameter('kp').get_parameter_value().double_array_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_array_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_array_value

        # self.kp = [10.0,10.0,10.0]
        # self.ki = [0.1,0.1,0.1]
        # self.kd = [0.1,0.1,0.1]

        self.sub_joint = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.pos_feedback = [0.0,0.0,0.0]
        self.pos_goal = [0.0,1.78,-1.78]
        self.pos_error = [0.0,0.0,0.0]
        self.pos_error_sum = [0.0,0.0,0.0]
        self.pos_error_diff = [0.0,0.0,0.0]
        self.pos_last_error = [0.0,0.0,0.0]
        # /forward_velocity_controller/commands
        self.pub_vel = self.create_publisher(Float64MultiArray, 'forward_velocity_controller/commands', 10)
        self.vel_control = [0.0,0.0,0.0]
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.sub_goal = self.create_subscription(Float64MultiArray, 'goal', self.goal_callback, 10)
        #pub state in bool
        self.pub_state = self.create_publisher(Bool, 'state', 10)
        self.state = Bool()


    def goal_callback(self, msg):
        self.pos_goal = msg.data

    def joint_callback(self, msg):

        self.pos_feedback = msg.position[0:3]

    def doPID(self):
        # self.get_logger().info('pos_goal: %s' % str(self.pos_goal))
        # self.get_logger().info('pos_feedback: %s' % str(self.pos_feedback))
        # self.get_logger().info('pos_error: %s' % str(self.pos_error))

        for i in range(3):
            self.pos_error[i] = self.pos_goal[i] - self.pos_feedback[i]
            self.vel_control[i] = self.kp[i]*self.pos_error[i] + self.ki[i]*self.pos_error_sum[i] + self.kd[i]*self.pos_error_diff[i]
            self.pos_error_sum[i] += self.pos_error[i]*0.1
            self.pos_error_diff[i] = (self.pos_error[i] - self.pos_last_error[i])/0.1
            self.pos_last_error[i] = self.pos_error[i]

        self.state.data = False
        if self.pos_error[0] < 0.1 and self.pos_error[1] < 0.1 and self.pos_error[2] < 0.1:
            self.pos_error_sum = [0.0,0.0,0.0]
            self.pos_error_diff = [0.0,0.0,0.0]
            self.state.data = True
        
    def timer_callback(self):
        self.doPID()
        msg = Float64MultiArray()
        msg.data = self.vel_control
        self.pub_vel.publish(msg)
        self.pub_state.publish(self.state)


def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
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
