#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from fams_interfaces.msg import RFID
import serial


class MyNode_1(Node):

    def __init__(self):
        super().__init__("RFID_node")
        self.get_logger().info("RFID node started")
        self.RFID_publisher = self.create_publisher(RFID, "RFID_Topic", 10)
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        self.read_raspberry_pi_()

    def read_raspberry_pi_(self):
        while True:
            msg = RFID()
            a = self.ser.readline().strip()  # Remove leading/trailing whitespace
            msg.rfid_code = a.decode('utf-8')  #bytes to string
            self.RFID_publisher.publish(msg)
            self.get_logger().info(msg.rfid_code)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode_1()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
