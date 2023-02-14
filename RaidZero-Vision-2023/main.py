# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import cv2 as cv
import numpy as np
#import cscore as cs
#from cscore import CameraServer
import threading
import logging
import pupil_apriltags
from imutils.video import VideoStream
import argparse
import datetime
import time

from networktables import NetworkTables
from networktables.util import ChooserControl

from AprilTagVisionHelper import AprilTagVisionHelper
from ConeVisionHelper import ConeVisionHelper
from AprilTagCapture import ATagCapture
from ConeCapture import ConeCapture

tablename = "SmartDashboard"
ip_address = "10.42.53.2"
aprilTag_size = 0.1651
cameramtx_filename = "./images/cameramtx.npz"

path_1 = '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-video-index0'
path_2 = '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-video-index0'
path_3 = '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0'

cone_camera_path = path_1

frame_width = 640
frame_height = 480

area_threshold = 10000

yellow_lower = [0, 56, 152]
yellow_upper = [76, 238, 255]

aprilTagVisionHelper = AprilTagVisionHelper(ip_address, cone_camera_path)
coneVisionHelper = ConeVisionHelper(ip_address, cone_camera_path, yellow_lower, yellow_upper, frame_width=frame_width, frame_height=frame_height, area_threshold=area_threshold)

NetworkTables.initialize()
sd = NetworkTables.getTable("SmartDashboard")

print("[INFO] detecting AprilTags...")

local_clock = 0
remote_clock = 0

def sync_networktables_time(table, key, value, isNew):
    global local_clock
    global remote_clock
    if key == "robottime":
        local_clock = time.perf_counter()
        remote_clock = value
        sd.putNumber("RPitime", value)

def gen_frames(aprilTagVisionHelper, coneVisionHelper):
    while(True):
        aprilTagVisionHelper.processVideos(drawAxes=True,drawMask=True)
        aprilTagVisionHelper.outputVideo()
        coneVisionHelper.processVideo(draw_mask=True, draw_marker=True, draw_rectangle=True)
        coneVisionHelper.outputVideo()

        key = cv.waitKey(1) & 0xFF
        if key == ord("q"):
            break

if __name__ == '__main__':
    num_cameras = len(aprilTagVisionHelper.getSinks())
    for (camera_num, cv_sink) in zip(range(num_cameras),aprilTagVisionHelper.getSinks()):
        print("Adding apriltag capture " + str(camera_num))
        aprilTagVisionHelper.addAprilTagCapturing(ATagCapture(aprilTagSize=aprilTag_size,
                                                      cameramtx_filename=cameramtx_filename, cvSink=cv_sink))
    coneVisionHelper.addConeCapture(ConeCapture(cv_sink=coneVisionHelper.getSink(),
                                                yellow_lower=coneVisionHelper.getYellowLower(),
                                                yellow_upper=coneVisionHelper.getYellowUpper(),
                                                frame_width=coneVisionHelper.getFrameWidth(),
                                                frame_height=coneVisionHelper.getFrameHeight(),
                                                area_threshold=coneVisionHelper.getAreaThreshold()))
    sd.addEntryListener(sync_networktables_time,key="robottime",immediateNotify=True)
    aprilTagVisionHelper.syncTimes()
    coneVisionHelper.syncTimes()
    gen_frames(aprilTagVisionHelper, coneVisionHelper)

input("Press enter to exit...")

cv.destroyAllWindows()
