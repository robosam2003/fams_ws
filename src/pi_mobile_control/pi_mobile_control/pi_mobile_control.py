from math import cos, sin
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Point
import serial

class pi_control(Node):

    def __init__(self):
        #Inherits Node functionality
        super().__init__('sam_bot')

        # Create a publisher that publishes the joint states so rivz can be run to visulaise system
        self.wheel_states_publisher = self.create_publisher(JointState, 'wheel_states', 10)

        # Create a subscriber that subscribes to the cmd_vel topic
        self.cmd_vel_subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Initialize the joint states for rviz
        self.wheel_states = JointState()
        self.wheel_states.name = ['upper_left_wheel_joint', 'upper_right_wheel_joint', 'lower_left_wheel_joint', 'lower_right_wheel_joint']
        self.wheepi_control_logger().info('pi_control node has been started')

    def cmd_vel_callback(self, msg):
        
        #Calculate the linear and angular velocities
        linear_velocity = msg.linear.x # This is the linear velocity in m/s
        angular_velocity = msg.angular.z  # This is the angular velocity in rad/s


# Define the main function
def main(args=None):
    # Initialize the ROS 2 node
    rclpy.init(args=args)

    # Create an instance of the SamBot class
    pi_control = pi_control()

    # Spin the node
    rclpy.spin(pi_control)

    # Destroy the node
    pi_control.destroy_node()

    # Shutdown ROS 2
    rclpy.shutdown()
