#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode_1(Node):

    def __init__(self):
        super().__init__("first_node")
        self.get_logger().info("hello from ros2")


def main(args=None):
    rclpy.init(args=args)

    # node will be created here. 
    # we can also run multiple nodes from here
    
    node = MyNode_1()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()