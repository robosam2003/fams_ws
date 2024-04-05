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


class MapOdomPublisher(Node):

    def __init__(self):
        super().__init__('map_odom_tf2_publisher')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # ROS timer to periodically call the `broadcast` method
        self.create_timer(0.05, self.broadcast)

        self.aruco_sub = self.create_subscription(Point, 
                                                  'nexus/aruco_tf',
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
        # MAP->ODOM transformation
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = 2.5
        t.transform.translation.y = 1.5
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0, 0, 0.5 * math.pi)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

        # ODOM->BASE_LINK transformation - This would eventually be fed from the Aruco marker node
        x = self.current_aruco.x
        y = self.current_aruco.y
        yaw = self.current_aruco.z
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'nexus/base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.05 #(wheel radius)   # (SAM BOT):0.15 # 0.1 (wheel radius) + 0.05 (wheel_zoff)
        q = quaternion_from_euler(0, 0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
        
        # # # Broadcast the tf for the map to mover6 base link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        # Add a namespace to the child frame id to avoid conflicts
        t.child_frame_id = 'mover6/base_link'
        t.transform.translation.x = 0.35
        t.transform.translation.y = 2.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(t)

        


def main():
    rclpy.init()
    node = MapOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()