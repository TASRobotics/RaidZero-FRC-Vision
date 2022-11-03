# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import cv2
import numpy as np
import cscore as cs
from cscore import CameraServer
import math
import logging
import numpy as np



logging.basicConfig(level=logging.DEBUG)
import networktables
nt = networktables.NetworkTablesInstance()
nt.initialize(server='10.42.53.2')


FRAME_WIDTH = 320
FRAME_HEIGHT = 240
#sets lower threshold for blue color
blue_lower = np.array([92 , 70 , 60] , dtype = 'uint8');

 #sets upper threshold for blue color
blue_upper = np.array([106, 240 , 255] , dtype = 'uint8');

#sets lower threshold for red color
red_lower = np.array([175 , 70 , 60] , dtype = 'uint8');
lower_red_lower = red_lower.copy()
lower_red_lower[0] = 0

 #sets upper threshold for blue color
red_upper = np.array([180, 240 , 255] , dtype = 'uint8');
lower_red_upper = red_upper.copy()
lower_red_upper[0]=5

ball_delta = .3

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

def main():
    frame = np.zeros(shape=(FRAME_WIDTH, FRAME_HEIGHT, 3), dtype=np.uint8)
    while True:
        ret, frame = cvSink.grabFrame(frame)
        blurred = cv2.GaussianBlur(frame, (11,11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
        (blue_contours, _) = cv2.findContours(blue_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        
        red_mask = cv2.inRange(hsv, red_lower, red_upper)
        lower_red_mask = cv2.inRange(hsv, lower_red_lower, lower_red_upper)
        red_mask = np.maximum(red_mask,lower_red_mask)
        (red_contours, _) = cv2.findContours(red_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        (lower_red_contours, _) = cv2.findContours(lower_red_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        

        if len(blue_contours+red_contours) > 0:
            for contour in blue_contours+red_contours:
                # bbox = cv2.boundingRect(contour)
                # rects = np.int0(cv2.boxPoints(bbox))
                ((x,y), radius) = cv2.minEnclosingCircle(contour)
                # M = cv2.moments(contour)
                # center = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
                # print("found color")
                # ball_cond = bbox[2]*bbox[3] > 2000
                # ball_cond = ball_cond & (abs(bbox[2]/bbox[3]-1)<ball_delta)
                overlap = cv2.contourArea(contour)/(math.pow(radius,2)*math.pi)
                ball_cond = radius>10
                ball_cond = ball_cond & (abs(overlap-1)<ball_delta)
                if ball_cond:
                    # cv2.rectangle(frame,(bbox[0],bbox[1]),(bbox[0]+bbox[2],bbox[1]+bbox[3]),(0,255,0),2)
                    # print("found ball")

                    print(abs(overlap-1))
                    cv2.circle(frame,(int(x),int(y)),int(radius),(0,255,0),2)
                
        #cv2.imshow("Tracking... ", frame)
        print(hsv[int(FRAME_HEIGHT/2),int(FRAME_WIDTH/2)])
        outputStream.putFrame(frame)
            
        # key = cv2.waitKey(1) & 0xFF        
        # # if the `q` key was pressed, break from the loop
        # if key == ord("q"):
        #     break


if __name__ == "__main__":

    import logging
    logging.basicConfig(level=logging.DEBUG)

    import networktables
    networktables.NetworkTables.initialize(server='10.42.53.2')

    main()

# th = threading.Thread(target=video_thread, daemon=True)
# th.start()

#mjpegServer = cs.MjpegServer("httpserver", 8081)
#mjpegServer.setSource(camera)

# print("mjpg server listening at http://0.0.0.0:8081")
# input("Press enter to exit...")

# cv2.destroyAllWindows()