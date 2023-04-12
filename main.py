# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import cv2 as cv
# import numpy as np
# #import cscore as cs
# #from cscore import CameraServer
# import threading
# import logging
# import pupil_apriltags
# from imutils.video import VideoStream
# import argparse
# import datetime
import time


from networktables import NetworkTables
# from networktables.util import ChooserControl

# from AprilTags.AprilTagCapture import ATagCapture
from DataFitting.LimitSwitchDetection import WristAlignment
from VisionHelper import NetworkTablesVisionHelper
from DataFitting.AbsoluteEncoderZeros import CalculateZeros

tablename = "SmartDashboard"
ip_address = "10.42.53.2"
aprilTag_size = 0.1524
# aprilTag_size = 0.101
cameramtx_path = "./AprilTags/Calibration/"



NetworkTables.initialize(ip_address)
sd = NetworkTables.getTable(tablename)


wristSafety = WristAlignment(ip_address, tablename)
visionHelper = NetworkTablesVisionHelper(ip_address, tablename)
armSafety = CalculateZeros(ip_address, tablename)
#logging.basicConfig(level=logging.DEBUG)
#import networktables
#nt = networktables.NetworkTablesInstance()
#nt.initialize(server='10.42.53.2')


FRAME_WIDTH = 640
FRAME_HEIGHT = 480

area_threshold = 12000

yellow_lower = [104, 0, 0]
yellow_upper = [170, 255, 255]

CONE_DETECTION = True

#cs = CameraServer.getInstance()
#cs.enableLogging()

#camera = cs.startAutomaticCapture()

#camera.setResolution(FRAME_WIDTH, FRAME_HEIGHT)

#camera = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, 640, 480, 30)

# Get a CvSink. This will capture images from the camera
#cvSink = cs.getVideo()

# (optional) Setup a CvSource. This will send images back to the Dashboard
# outputStream = cs.putVideo("Rectangle", FRAME_WIDTH, FRAME_HEIGHT)
print("[INFO] detecting AprilTags...")
# Allocating new images is very expensive, always try to preallocate

#windowName = "Live video Feed"
#cv2.namedWindow(windowName)
local_clock = 0
remote_clock = 0

def sync_networktables_time(table, key, value, isNew):
    global local_clock
    global remote_clock
    if key == "robottime":
        local_clock = time.perf_counter()
        remote_clock = value
        sd.putNumber("RPitime", value)
    



def start_cycles(visionHelper):
    while(True):
        visionHelper.processVideos(drawAxes=True, drawMask=True, drawMarker=True, drawRectangle=True)
        visionHelper.outputVideo()
        wristSafety.run()
        armSafety.run()


        #output = frame.tobytes()
        #yield (b'--frame\r\n'
        #           b'Content-Type: image/jpeg\r\n\r\n' + output + b'\r\n')  # concat frame one by one and show result
        #cv2.imshow("Trackin  g... ", frame)
        #cv2.imshow("Gray", gray)
        #cv2.imshow(windowName, frame)
        key = cv.waitKey(1) & 0xFF
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break

if __name__ == '__main__':
    num_cameras = len(visionHelper.getSinks())
    # if CONE_DETECTION:
    visionHelper.initializeDetectors(1, aprilTag_size, cameramtx_path, area_threshold, yellow_lower, yellow_upper)
    # visionHelper.initializeAprilTagDetect(aprilTag_size , cameramtx_path)
    # visionHelper.initializeConeDetect(area_threshold, yellow_lower, yellow_upper)
    sd.addEntryListener(sync_networktables_time,key="robottime",immediateNotify=True)
    visionHelper.syncTimes()
    start_cycles(visionHelper=visionHelper)
    
    #app.run(debug=True,host="0.0.0.0")

#th = threading.Thread(target=video_thread, daemon=True)
#th.start()

#mjpegServer = cs.MjpegServer("httpserver", 8081)
#mjpegServer.setSource(camera)

#print("mjpg server listening at http://0.0.0.0:8081")
input("Press enter to exit...")

cv.destroyAllWindows()
