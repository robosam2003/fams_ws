#!/usr/bin/env python3


# Basic ROS 2 program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
   
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import os
import numpy as np
  
class ArucoReader(Node):

  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('aruco_reader')
    self.get_logger().info('Aruco Reader Node Started')
       
    self.location_publisher = self.create_publisher("Mobile_Locations",locations)
       
    # We will publish a message every 0.1 seconds
    timer_period = 0.1  # seconds
       
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)
          
    # Create a VideoCapture object
    self.cap = cv2.VideoCapture(-1)
  
    self.br = CvBridge()
    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    aruco_type = "DICT_4X4_100" #Is looking for 4x4 only

    intrinsic_camera = np.array(((1.51676623e+03,0,9.58518895e+02),(0, 1.59055282e+03, 5.44395560e+02),(0,0,1)))
    distortion = np.array((1.73228514e-02, -7.03010353e-01, -7.57199459e-04,  6.85156948e-02, 8.77224638e-01))
    os.system('v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1')
    os.system('v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_automatic=0')

    while self.cap.isOpened():
	
      os.system('v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=312')
      os.system('v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature=3700')

      ret, img = self.cap.read()
      output, locations = self.pose_estimation(img, ARUCO_DICT[aruco_type], intrinsic_camera, distortion)
      cv2.imshow('Estimated Pose', output)
      self.location_publisher.publish(locations)
        

      key = cv2.waitKey(1) & 0xFF
      if key == ord('q'):
        break

    self.cap.release()
    cv2.destroyAllWindows()

  def pose_estimation(self, frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
      cameraMatrix=matrix_coefficients,
      distCoeff=distortion_coefficients)

    if len(corners) > 0:
      for i in range(0, len(ids)):
            
        rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.15, matrix_coefficients, distortion_coefficients)            
        cv2.aruco.drawDetectedMarkers(frame, corners,ids) 
        origin=[0.00649249, -0.01326193,  2.86440854]
        location=tvec-origin
        cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.05)  
    
    return frame, location



ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
}




    

   
def main(args=None):
  rclpy.init(args=args)
  aruco_reader = ArucoReader()
  rclpy.spin(aruco_reader)
  aruco_reader.destroy_node()
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()