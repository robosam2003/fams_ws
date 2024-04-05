# import numpy as np

# from pykin.robots.single_arm import SingleArm
# from pykin.kinematics import transform as t_utils
# from pykin.utils import plot_utils as p_utils

# urdf_path = "src/mover6_description/src/description/CPRMover6WithGripper.urdf.xacro"

# robot = SingleArm(urdf_path, t_utils.Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0]))

# target_thetas = np.array(
#     [np.random.uniform(-np.pi, np.pi) for _ in range(robot.arm_dof)]
# )
# init_thetas = np.random.randn(robot.arm_dof)
# robot.setup_link_name("gripperFinger1", "mover6_gripper")

# robot.show_robot_info()

import rclpy
from rclpy.node import Node
import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, PoseStamped, Pose
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D


class Mover6(Node):
    def __init__(self):
        super().__init__('mover6')
        self.get_logger().info('Mover6 node has been initialized.')
        self.my_chain = ikpy.chain.Chain.from_urdf_file("src/mover6_description/src/description/CPRMover6WithGripperIKModel.urdf.xacro")
        self.target_position = [0.2, 0.0, 0.3]


        self.joint_state_publisher = self.create_publisher(JointState, 'mover6_joint_states', 10)

        self.point_publisher = self.create_publisher(PoseStamped, 'mover6_target_position', 10)

        # timer 
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        joint_state = JointState()
        joint_state.name = ['Joint0', 'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5']
        ik = self.my_chain.inverse_kinematics(self.target_position)
        # self.get_logger().info(self.my_chain.__repr__())

        
        ik_copy = ik.copy()

        
        # reverse it
        # ik = ik[::-1]
        joint_state.position = [ik[i+1] for i in range(6)]
        # self.get_logger().info('Publishing: "%s"' % joint_state)
        self.joint_state_publisher.publish(joint_state)

        # real_frame = self.my_chain.forward_kinematics(np.zeros(9))
        real_frame = self.my_chain.forward_kinematics(ik)
        # self.get_logger().info("Computed position vector : %s, original position vector : %s" % (real_frame[:3, 3], self.target_position))

        point = PoseStamped()
        point.header.frame_id = "mover6/base_link"
        point.header.stamp = self.get_clock().now().to_msg()
        point.pose.position.x = self.target_position[0]
        point.pose.position.y = self.target_position[1]
        point.pose.position.z = self.target_position[2]

        self.point_publisher.publish(point)


def main(args=None):
    rclpy.init(args=args)

    mover6 = Mover6()

    rclpy.spin(mover6)

    mover6.destroy_node()
    rclpy.shutdown()