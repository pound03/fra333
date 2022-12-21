#!/usr/bin/python3

from package_name.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
import sys

class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')

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
