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
import apriltag
from imutils.video import VideoStream
from flask import Response
from flask import Flask
from flask import render_template
import argparse
import datetime
import imutils
import time

from networktables import NetworkTables
from networktables.util import ChooserControl

from scipy.spatial.transform import Rotation as R


app = Flask(__name__)

NetworkTables.initialize("10.6.21.129")
sd = NetworkTables.getTable("SmartDashboard")
#logging.basicConfig(level=logging.DEBUG)
#import networktables
#nt = networktables.NetworkTablesInstance()
#nt.initialize(server='10.42.53.2')


FRAME_WIDTH = 640
FRAME_HEIGHT = 480

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
    

def draw(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1,2)
    # draw ground floor in green
    img = cv.drawContours(img, [imgpts[:4]],-1,(0,255,0),-3)
    # draw pillars in blue color
    for i,j in zip(range(4),range(4,8)):
        img = cv.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)
    # draw top layer in red color
    img = cv.drawContours(img, [imgpts[4:]],-1,(0,0,255),3)
    return img




def gen_frames(cameramtx_filename,camera_number):
    global local_clock
    global remote_clock
    cvSink = cv.VideoCapture(camera_number)
    cvSink.set(cv.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cvSink.set(cv.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    #Load the camera matrix
    with np.load(cameramtx_filename) as X:
        mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]

    #Create AprilTag Detector
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    toc = time.perf_counter()
    while True:
        success, frame = cvSink.read()
        results = []
        if not success:
            break
        else:
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            tic = time.perf_counter()
            capturetime = f"Printed and captured image in {tic - toc:0.4f} seconds"
            results = detector.detect(gray)
            #print(frame.shape[:-1])
            t_text = ""
            r_text = ""

            if len(results) > 0:
                print("In results")
                for r in results:
                    # bbox = cv2.boundingRect(contour)
            		# extract the bounding box (x, y)-coordinates for the AprilTag
    		        # and convert each of the (x, y)-coordinate pairs to integers
                    (ptA, ptB, ptC, ptD) = r.corners
                    ptB = (int(ptB[0]), int(ptB[1]))
                    ptC = (int(ptC[0]), int(ptC[1]))
                    ptD = (int(ptD[0]), int(ptD[1]))
                    ptA = (int(ptA[0]), int(ptA[1]))
                    # draw the bounding box of the AprilTag detection
                    cv.line(frame, ptA, ptB, (0, 255, 0), 2)
                    cv.line(frame, ptB, ptC, (0, 255, 0), 2)
                    cv.line(frame, ptC, ptD, (0, 255, 0), 2)
                    cv.line(frame, ptD, ptA, (0, 255, 0), 2)
                    # draw the center (x, y)-coordinates of the AprilTag
                    (cX, cY) = (int(r.center[0]), int(r.center[1]))
                    cv.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
                    # draw the tag family on the image
                    tagFamily = r.tag_family.decode("utf-8")
                    # cv2.putText(frame, "Point A", (ptA[0], ptA[1] - 15),
                    # 	cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    # cv2.putText(frame, "Point B", (ptB[0], ptB[1] - 15),
                    # 	cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    # cv2.putText(frame, "Point C", (ptC[0], ptC[1] - 15),
                    # 	cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    # cv2.putText(frame, "Point D", (ptD[0], ptD[1] - 15),
                    # 	cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    squaresize=5.3
                    square = np.array([[-squaresize/2,squaresize/2,0],
                        [+squaresize/2,squaresize/2,0],
                        [+squaresize/2,-squaresize/2,0],
                        [-squaresize/2,-squaresize/2,0]])

                    reorient = np.flip(r.corners,0)
                    mask, rvecs, tvecs = cv.solvePnP(square, reorient,
                        mtx, dist, flags = cv.SOLVEPNP_IPPE_SQUARE)
                    cv.drawFrameAxes(frame, mtx, dist, rvecs, tvecs, 3)
                    to_euler = R.from_rotvec(rvecs.reshape(3,))
                    sd.putNumberArray("Rotation", to_euler.as_euler('yxz',degrees=True))
                    sd.putNumberArray("Translation", tvecs)
                    sd.putNumber("TimeDiff", local_clock-remote_clock)
                    sd.putNumber("Cameratime", remote_clock+time.perf_counter()-local_clock)
                    t_text += "Translation " + str(to_euler.as_euler('zxy',degrees=True))
                    r_text += "Rotation " +str(rvecs)
#                    matrix_text += "No Matrix, original points: \n" + str(april_corners)

            findtime = f"Found tags and orientation in {toc - tic:0.4f} seconds"
            cv.putText(frame,r_text,(5,45),cv.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
            cv.putText(frame,t_text,(5,60),cv.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
            cv.putText(frame,capturetime,(5,15),cv.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
            cv.putText(frame,findtime,(5,30),cv.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
            resolution = "Resolution: " + str(frame.shape[:-1]) 
            cv.putText(frame,resolution,(5,frame.shape[0]-5),cv.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
            toc = time.perf_counter()

            ret, buffer = cv.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result



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


@app.route('/video_feed')
def video_feed():
    #Video streaming route. Put this in the src attribute of an img tag
    return Response(gen_frames("./images/cameramtx.npz",0), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')


if __name__ == '__main__':
    sd.addEntryListener(sync_networktables_time,key="robottime",immediateNotify=True)
    app.run(debug=True,host="0.0.0.0")

#th = threading.Thread(target=video_thread, daemon=True)
#th.start()

#mjpegServer = cs.MjpegServer("httpserver", 8081)
#mjpegServer.setSource(camera)

#print("mjpg server listening at http://0.0.0.0:8081")
input("Press enter to exit...")

cv.destroyAllWindows()
