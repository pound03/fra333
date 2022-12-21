#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
import numpy as np
from enum import Enum
import sys

class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')
        publish_topic = "/forward_velocity_controller/commands"
        self.publisher_ = self.create_publisher(Float64MultiArray, publish_topic, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [10.0, 0.0 , 0.0]
        self.publisher_.publish(msg)

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
