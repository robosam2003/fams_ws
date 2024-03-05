import numpy as np
import cv2
import sys
import time
import os

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	
}



def aruco_display(corners, ids, rejected, image):
    
	
	if len(corners) > 0:
		
		ids = ids.flatten()
		
		for (markerCorner, markerID) in zip(corners, ids):
			
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners
			
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))

			cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
			cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
			cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
			
			
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
			
			cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)
			# cv2.putText(image, str(cX),(topLeft[0], topLeft[1] + 20), cv2.FONT_HERSHEY_SIMPLEX,
			# 	0.7, (0, 0, 255), 2)
			# cv2.putText(image, str(cY),(topLeft[0], topLeft[1] + 35), cv2.FONT_HERSHEY_SIMPLEX,
			# 	0.7, (0, 0, 255), 2)
			# print("[Inference] ArUco marker ID: {}".format(markerID))
			# print("[Inference] X Co-ord: {}".format(cX))
			# print("[Inference] Y Co-ord: {}".format(cY))
			
		
	return image



def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

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


    

aruco_type = "DICT_4X4_100" #Is looking for 4x4 only

arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])
arucoParams = cv2.aruco.DetectorParameters_create()

intrinsic_camera = np.array(((1.51676623e+03,0,9.58518895e+02),(0, 1.59055282e+03, 5.44395560e+02),(0,0,1)))
distortion = np.array((1.73228514e-02, -7.03010353e-01, -7.57199459e-04,  6.85156948e-02, 8.77224638e-01))

cap = cv2.VideoCapture(-1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

print(width, height, fps)

os.system('v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1')
os.system('v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_automatic=0')

while cap.isOpened():
	
	os.system('v4l2-ctl -d /dev/video0 --set-ctrl=exposure_time_absolute=312')
	os.system('v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature=3700')

	ret, img = cap.read()
	# grey=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


	# UDimg=cv2.undistort(grey,intrinsic_camera,distortion)


	
	# corners, ids, rejected = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)
	
	

	# detected_markers = aruco_display(corners, ids, rejected,img)

	# cv2.imshow("Image",detected_markers)
	
	
	output, location = pose_estimation(img, ARUCO_DICT[aruco_type], intrinsic_camera, distortion)
	
	# cv2.rectangle(img,(5,5),(1920,1080),(0,255,0),3)
	# cv2.line(img,(0,540),(1920,540),(0,255,0),1)
	# cv2.line(img,(960,0),(960,1080),(0,255,0),1)
	# # cv2.putText(img, '960,540',(960,540), cv2.FONT_HERSHEY_SIMPLEX,
	# # 			0.5, (0, 0, 255), 2)
	# cv2.putText(img, '0,0',(0,10), cv2.FONT_HERSHEY_SIMPLEX,
	# 			0.5, (0, 0, 255), 2)


	cv2.imshow('Estimated Pose', output)
    

	key = cv2.waitKey(1) & 0xFF
	if key == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()