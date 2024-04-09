import math

from geometry_msgs.msg import TransformStamped, Point

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage



def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class FamsTFBroadcaster(Node):
    def __init__(self):
        super().__init__('map_odom_tf2_publisher')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_name', 'nexus_PH'),
                ('robot_type', 'sam_bot'), # Could be a sam bot (nexus bot/ diffdrive thing) or a mover6 (6DOF arm)
                ('initial_base_link_pos', "0.0 0.0 0.0") # x, y, yaw
            ]
        )

        self.robot_name = self.get_parameter('robot_name').value
        self.robot_type = self.get_parameter('robot_type').value
        self.initial_base_link_pos = self.get_parameter('initial_base_link_pos').value.split(' ') # x, y, yaw
        self.initial_base_link_pos = [float(i) for i in self.initial_base_link_pos]


        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # ROS timer to periodically call the `broadcast` method
        self.create_timer(0.05, self.broadcast)

        self.aruco_sub = self.create_subscription(Point, 
                                                  'aruco_tf',
                                                   self.aruco_callback,
                                                    10)
        
        self.get_logger().info('Map->Odom broadcaster has been started')
        self.current_aruco = Point()
        self.current_aruco.x = 0.0
        self.current_aruco.y = 0.0
        self.current_aruco.z = 0.0
        

    def aruco_callback(self, msg):
        self.current_aruco = msg    

    def broadcast(self):
        if self.robot_type == 'sam_bot':
            # MAP->ODOM transformation
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = self.robot_name + '/odom'
            t.transform.translation.x = self.initial_base_link_pos[0] # x
            t.transform.translation.y = self.initial_base_link_pos[1] # y
            t.transform.translation.z = 0.0
            q = quaternion_from_euler(0, 0, self.initial_base_link_pos[2])
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

            # ODOM->BASE_LINK transformation - This is fed from the aruco_tf topic
            x = self.current_aruco.x
            y = self.current_aruco.y
            yaw = self.current_aruco.z
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.robot_name + '/odom'
            t.child_frame_id = self.robot_name + '/base_link'
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.05 #(wheel radius)   # (SAM BOT):0.15 # 0.1 (wheel radius) + 0.05 (wheel_zoff)
            q = quaternion_from_euler(0, 0, yaw)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)
        
        elif self.robot_type == 'mover6':
            # # # Broadcast the tf for the map to mover6 base link
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            # Add a namespace to the child frame id to avoid conflicts
            t.child_frame_id = self.robot_name + '/base_link'
            t.transform.translation.x = self.initial_base_link_pos[0] # x
            t.transform.translation.y = self.initial_base_link_pos[1] # y
            t.transform.translation.z = 0.0
            yaw = self.initial_base_link_pos[2] # yaw
            q = quaternion_from_euler(0, 0, yaw)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(t)
        else: 
            self.get_logger().error('Unknown robot type. Please check the robot_type parameter')
        


def main():
    rclpy.init()
    node = FamsTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()