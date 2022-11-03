# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import cv2
import numpy as np
import cscore as cs
from cscore import CameraServer
import threading
import logging



logging.basicConfig(level=logging.DEBUG)
import networktables
nt = networktables.NetworkTablesInstance()
nt.initialize(server='10.42.53.2')


FRAME_WIDTH = 320
FRAME_HEIGHT = 240
#sets lower threshold for blue color
blue_lower = np.array([100 , 50 , 0] , dtype = 'uint8');

 #sets upper threshold for blue color
blue_upper = np.array([255, 150 , 100] , dtype = 'uint8');

#sets lower threshold for red color
red_lower = np.array([0 , 50 , 150] , dtype = 'uint8');

 #sets upper threshold for blue color
red_upper = np.array([100, 150 , 255] , dtype = 'uint8');

ball_delta = .5

cs = CameraServer.getInstance()
cs.enableLogging()

camera = cs.startAutomaticCapture()

#camera.setResolution(FRAME_WIDTH, FRAME_HEIGHT)

#camera = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, 640, 480, 30)

# Get a CvSink. This will capture images from the camera
cvSink = cs.getVideo()

# (optional) Setup a CvSource. This will send images back to the Dashboard
outputStream = cs.putVideo("Rectangle", FRAME_WIDTH, FRAME_HEIGHT)

# Allocating new images is very expensive, always try to preallocate


#capture = cv2.VideoCapture(0)

#capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

def video_thread():
    frame = np.zeros(shape=(FRAME_WIDTH, FRAME_HEIGHT, 3), dtype=np.uint8)
    while True:
        ret, frame = cvSink.grabFrame(frame)
        
        blue_threshold = cv2.inRange(frame, blue_lower, blue_upper)
        blue_blur = cv2.GaussianBlur(blue_threshold, (3,3), 0)
        (blue_contours, _) = cv2.findContours(blue_blur.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        
        red_threshold = cv2.inRange(frame, red_lower, red_upper)
        red_blur = cv2.GaussianBlur(red_threshold, (3,3), 0)
        (red_contours, _) = cv2.findContours(red_blur.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(blue_contours) > 0:
            for contour in blue_contours+red_contours:
                bbox = cv2.boundingRect(contour)
                # rects = np.int0(cv2.boxPoints(bbox))
                ball_cond = bbox[2]*bbox[3] > 2000
                ball_cond = ball_cond & (abs(bbox[2]/bbox[3]-1)<ball_delta)
                if ball_cond:
                    cv2.rectangle(frame,(bbox[0],bbox[1]),(bbox[0]+bbox[2],bbox[1]+bbox[3]),(0,255,0),2)
                
        #cv2.imshow("Tracking... ", frame)
        outputStream.putFrame(frame)
            
        key = cv2.waitKey(1) & 0xFF        
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break


th = threading.Thread(target=video_thread, daemon=True)
th.start()

#mjpegServer = cs.MjpegServer("httpserver", 8081)
#mjpegServer.setSource(camera)

print("mjpg server listening at http://0.0.0.0:8081")
input("Press enter to exit...")

cv2.destroyAllWindows()