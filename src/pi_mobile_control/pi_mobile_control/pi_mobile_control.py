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

        #Initiate Serial communication to arduino
        self.ser = serial.Serial('/dev/ttyACM0', 115200)

        # Create record of start time of system
        self.initial_time = self.get_clock().now().nanoseconds / 1e9

        # Initialize the joint states for rviz
        self.wheel_states = JointState()
        self.wheel_states.name = ['upper_left_wheel_joint', 'upper_right_wheel_joint', 'lower_left_wheel_joint', 'lower_right_wheel_joint']
        self.get_logger().info('pi_control node has been started')

    def cmd_vel_callback(self, msg):

        #Calculate the linear and angular velocities
        linear_velocity = msg.linear.x # This is the linear velocity in m/s
        angular_velocity = msg.angular.z  # This is the angular velocity in rad/s

        wheel_radius = 0.05
        wheel_separation = 0.3

        current_time = self.get_clock().now().nanoseconds / 1e9 # This is the current time in seconds
        elapsed_time = float(current_time - self.initial_time)

        #if elapsed_time > 0.1:
        #    elapsed_time = 0.01

        # Compute the left and right wheel velocities
        left_wheel_velocity = (linear_velocity - angular_velocity * wheel_separation / 2.0) / wheel_radius
        right_wheel_velocity = -(linear_velocity + angular_velocity * wheel_separation / 2.0) / wheel_radius
        
        left_wheel_velocity = round(left_wheel_velocity, 2)
        right_wheel_velocity = round(right_wheel_velocity, 2)

        self.ser.write(left_wheel_velocity)
        self.ser.write(right_wheel_velocity)

        # Compute the left and right wheel positions
        left_wheel_position = self.wheel_states.position[0] + left_wheel_velocity * elapsed_time
        right_wheel_position = self.wheel_states.position[1] + right_wheel_velocity * elapsed_time

        # Update the joint states
        self.wheel_states.position = [left_wheel_position, right_wheel_position, left_wheel_position, right_wheel_position]
        self.wheel_states.velocity = [left_wheel_velocity, right_wheel_velocity, left_wheel_velocity, right_wheel_velocity]

        # Publish the joint states - this will make the wheels spin
        self.wheel_states_publisher.publish(self.wheel_states)





# Define the main function
def main(args=None):
    # Initialize the ROS 2 node
    rclpy.init(args=args)

    # Create an instance of the SamBot class
    pi_mobile_control = pi_control()

    # Spin the node
    rclpy.spin(pi_mobile_control)

    # Destroy the node
    pi_mobile_control.destroy_node()

    # Shutdown ROS 2
    rclpy.shutdown()
