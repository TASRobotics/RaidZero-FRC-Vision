# -*- coding: utf-8 -*-
"""
Created on Fri Sep 16 14:54:12 2022

@author: bayntuna
"""

import numpy as np
import cv2 as cv
import glob

FOLDER = "./Camera 2 images/"
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 34, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob(FOLDER+'*.jpg')
for fname in images:
    img = cv.imread(fname)
    print(fname)
    img = cv.resize(img,[640,480])
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (9,6), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        print("found")
        cv.drawChessboardCorners(img, (9,6), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(100)

cv.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

for (fname, rvec, tvec) in zip(images,rvecs,tvecs):
    img = cv.imread(fname)
    print(fname)
    img = cv.resize(img,[640,480])
    
    cv.drawFrameAxes(img, mtx, dist, rvec, tvec, 1)    
    cv.imshow('img', img)
    cv.waitKey(1000)
            
        
    
img = cv.imread(images[0])
h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
np.savez(FOLDER+"cameramtx",ret=ret, mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
# undistort
dst = cv.undistort(img, mtx, dist, None, newcameramtx)
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imwrite(FOLDER+'calibresult.png', dst)
# undistort
mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
# cv.imwrite(FOLDER+'calibresult.png', dst)
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error
print( "total error: {}".format(mean_error/len(objpoints)) )

K_l = newcameramtx
K_r = K_l

img = cv.imread(images[1])
print(fname)
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
ret, pts_l = cv.findChessboardCorners(gray, (9,6), None)
img = cv.imread(images[2])
print(fname)
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
ret, pts_r = cv.findChessboardCorners(gray, (9,6), None)

pts_r = np.squeeze(pts_r)
pts_l = np.squeeze(pts_l)


pts_l_norm = cv.undistortPoints(np.expand_dims(pts_l, axis=1), cameraMatrix=newcameramtx, distCoeffs=None)
pts_r_norm = cv.undistortPoints(np.expand_dims(pts_r, axis=1), cameraMatrix=newcameramtx, distCoeffs=None)

E, mask = cv.findEssentialMat(pts_l_norm, pts_r_norm, focal=1.0, pp=(0., 0.), method=cv.RANSAC, prob=0.999, threshold=3.0)
points, R, t, mask = cv.recoverPose(E, pts_l_norm, pts_r_norm)

M_r = np.hstack((R, t))
M_l = np.hstack((np.eye(3, 3), np.zeros((3, 1))))

P_l = np.dot(K_l,  M_l)
P_r = np.dot(K_r,  M_r)
point_4d_hom = cv.triangulatePoints(P_l, P_r, np.expand_dims(pts_l, axis=1), np.expand_dims(pts_r, axis=1))
point_4d = point_4d_hom / np.tile(point_4d_hom[-1, :], (4, 1))
point_3d = point_4d[:3, :].T
