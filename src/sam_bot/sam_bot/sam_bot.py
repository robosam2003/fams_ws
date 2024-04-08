# This is a node that controls the SAM_BOT's wheels and subsequently moves the robot in the virtual space.
# INPUTS: 
# - cmd_vel: geometry_msgs/Twist
# OUTPUTS:
# - joint_states: sensor_msgs/JointState
# - aruco_tf: tf2_msgs/TFMessage

# In reality, ros2_control would handle all this. 

# Import the necessary Python libraries
from math import cos, sin
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Point

# Create a class that inherits from the Node class
class SamBot(Node):

    # Define the constructor of the class
    def __init__(self):
        # Call the constructor of the parent class
        super().__init__('sam_bot')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_name', 'nexus'),
            ]
        )
    
        robot_name = self.get_parameter('robot_name').value # Get the name of the robot from the parameter server

        # Create a publisher that publishes the joint states of the SAM_BOT
        self.wheel_states_publisher = self.create_publisher(JointState, 'wheel_states', 10)

        # Create an aruco_tf publisher
        self.aruco_tf_publisher = self.create_publisher(Point,
                                                        'aruco_tf',
                                                        10)

        # Create a subscriber that subscribes to the cmd_vel topic
        self.cmd_vel_subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        

        # Initialize the joint states
        self.wheel_states = JointState()
        # self.wheel_states.name = ['drivewhl_l_joint', 'drivewhl_r_joint']
        self.wheel_states.name = ['upper_left_wheel_joint', 'upper_right_wheel_joint', 'lower_left_wheel_joint', 'lower_right_wheel_joint']
        self.wheel_states.position = [0.0, 0.0, 0.0, 0.0]
        self.wheel_states.velocity = [0.0, 0.0, 0.0, 0.0]
        self.wheel_states.effort = [0.0, 0.0, 0.0, 0.0]

        self.initial_pose = [0.0, 0.0, 0.0]
        # self.robot_pos_mag_phase = [0.0, 0.0]
        self.robot_pos_xy_yaw = [0.0, 0.0, 0.0]
        self.initial_time = self.get_clock().now().nanoseconds / 1e9

        # Perform an initial joint states publication
        self.wheel_states_publisher.publish(self.wheel_states)

        # Perform an initial ARUCO publication
        aruco = Point()
        aruco.x = 0.0; aruco.y = 0.0; aruco.z = 0.0
        self.aruco_tf_publisher.publish(aruco)

        self.get_logger().info('sam_bot node has been started')
        

    # Define the callback function for the cmd_vel topic
    def cmd_vel_callback(self, msg):
        # Calculate the linear and angular velocities
        linear_velocity = msg.linear.x # This is the linear velocity in m/s
        angular_velocity = msg.angular.z  # This is the angular velocity in rad/s

        # wheel_radius = 0.1
        # wheel_separation = 0.4
        wheel_radius = 0.05
        wheel_separation = 0.3 + 0.0505*2
        acceleration = 0.1
        current_time = self.get_clock().now().nanoseconds / 1e9 # This is the current time in seconds
        elapsed_time = float(current_time - self.initial_time)
        if elapsed_time > 0.1:
            elapsed_time = 0.01

        # Compute the left and right wheel velocities
        left_wheel_velocity = (linear_velocity - angular_velocity * wheel_separation / 2.0) / wheel_radius
        right_wheel_velocity = -(linear_velocity + angular_velocity * wheel_separation / 2.0) / wheel_radius

        # Compute the left and right wheel positions
        left_wheel_position = self.wheel_states.position[0] + left_wheel_velocity * elapsed_time
        right_wheel_position = self.wheel_states.position[1] + right_wheel_velocity * elapsed_time

        # Update the joint states
        self.wheel_states.position = [left_wheel_position, right_wheel_position, left_wheel_position, right_wheel_position]
        self.wheel_states.velocity = [left_wheel_velocity, right_wheel_velocity, left_wheel_velocity, right_wheel_velocity]
        
        # Publish the joint states - this will make the wheels spin
        self.wheel_states_publisher.publish(self.wheel_states)

        # Infer the robot's position in 2D
        displacement = float(linear_velocity * elapsed_time)
        change_in_angle = float(angular_velocity * elapsed_time)

        initial_theta = self.robot_pos_xy_yaw[2]

        displacement_x = displacement * cos(initial_theta)
        displacement_y = displacement * sin(initial_theta)

        # Update X
        self.robot_pos_xy_yaw[0] += displacement_x
        # Update Y
        self.robot_pos_xy_yaw[1] += displacement_y
        # Update Yaw
        self.robot_pos_xy_yaw[2] += change_in_angle

        # Update the initial time
        self.initial_time = current_time

        # self.get_logger().info('Robot position: x = %f, y = %f, yaw = %f' % (self.robot_pos_xy_yaw[0], self.robot_pos_xy_yaw[1], self.robot_pos_xy_yaw[2]))

        aruco = Point()
        aruco.x = float(self.robot_pos_xy_yaw[0])
        aruco.y = float(self.robot_pos_xy_yaw[1])
        aruco.z = float(self.robot_pos_xy_yaw[2]) # This is the yaw angle in radians

        self.aruco_tf_publisher.publish(aruco)




# Define the main function
def main(args=None):
    # Initialize the ROS 2 node
    rclpy.init(args=args)

    # Create an instance of the SamBot class
    sam_bot = SamBot()

    # Spin the node
    rclpy.spin(sam_bot)

    # Destroy the node
    sam_bot.destroy_node()

    # Shutdown ROS 2
    rclpy.shutdown()
