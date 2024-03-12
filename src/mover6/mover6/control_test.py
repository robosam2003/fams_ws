import rclpy
from rclpy.node import Node

from fams_interfaces.msg import Mover6Control

class Mover6ControlTest(Node):
    def __init__(self):
        super().__init__('mover6_control_test')

        self.joint_positions_publisher = self.create_publisher(
            Mover6Control,
            'mover6_control',
            10
        )

        while True:
            # Get user input, with comma separated values
            joint_angles = input("Enter joint angles (comma separated values): ")
            joint_angles = joint_angles.split(',')
            joint_angles = [float(joint_angle) for joint_angle in joint_angles]

            # Create message
            msg = Mover6Control()
            msg.joint_angles = joint_angles
            # Publish message
            self.joint_positions_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    mover6_control_test = Mover6ControlTest()

    rclpy.spin(mover6_control_test)

    mover6_control_test.destroy_node()
    rclpy.shutdown()