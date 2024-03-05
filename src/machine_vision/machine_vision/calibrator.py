import numpy as np
import cv2 as cv
import glob
import os
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((4*5,3), np.float32)
objp[:,:2] = np.mgrid[0:5,0:4].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
counter=0
calibrate_path = '/home/uos/fams_ws/src/machine_vision/machine_vision/Calibrate'
images = os.listdir(calibrate_path)
for fname in images:
    print(fname)
    path = os.path.join(calibrate_path, fname)
    img = cv.imread(path)

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (5,4), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        counter=counter+1
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (5,4), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(100)
cv.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print(counter)
print(mtx)
print(dist)
print(rvecs)
print(tvecs)

calibrate_path1 = '/home/uos/fams_ws/src/machine_vision/machine_vision/Calibrate'
images1 = os.listdir(calibrate_path1)
path1 = os.path.join(calibrate_path1, 'cal1.jpg')
img1 = cv.imread(path1)



h,  w = img1.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

# undistort
dst = cv.undistort(img1, mtx, dist, None, newcameramtx)
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
a= cv.imwrite('calibresult.jpg', dst)
cv.imshow('undistorted',a)
cv.waitKey(1000)

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error
print( "total error: {}".format(mean_error/len(objpoints)) )