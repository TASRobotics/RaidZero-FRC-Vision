# -*- coding: utf-8 -*-
"""
Created on Fri Oct 28 20:10:50 2022

@author: bayntuna
"""

import cv2 as cv
import numpy as np
import apriltag
import imutils
from networktables import NetworkTables
from cscore import CameraServer
import time

from scipy.spatial.transform import Rotation as R


class NetworkTablesHelper:
    
    #Constructor of Helper
        
    def __init__(self, ip_address, cameras = 1, FRAME_WIDTH=640, FRAME_HEIGHT=480):
        NetworkTables.initialize(ip_address)
        cs = CameraServer.getInstance()
        self.usbcams = []
        self.cvsinks = []
        cameranames = []
        self.apriltagcaptures = []
        self.timediff = 0
        
        for camera in range(cameras):
            self.usbcams.append(cs.startAutomaticCapture())
            self.usbcams[-1].setResolution(FRAME_WIDTH,FRAME_HEIGHT)
            cameranames.append(usbcams[-1].name)
            self.cvsinks.append(cs.getVideo(cameranames[-1]))
        
        
        self.sd = NetworkTables.getTable("SmartDashboard")
        self.sd.putStringArray("CameraNames", cameranames)

    
        
    def getSinks(self):
        return self.cvsinks
    
    def syncTimes(self):
        self.timediff = self.sd.getNumber("RobotTime", 0) - time.perf_counter()
        
        
    def addAprilTagCapturing(self, atagcapturing):
        self.apriltagcaptures.append(atagcapturing)
        
    
    def processVideos(self, drawAxes = False, drawMask = False):
        for atagcapture in self.apriltagcaptures:
            if atagcapture.captureFrame() > 0 and atagcapture.genResults():
                atagcapture.drawAxes() if drawAxes
                atagcapture.drawMask() if drawMask
                cameraname = atagcapture.getSink().name
                translation, rotation, reprojerr, timestamp = atagcapture.getTranslationsAngles(degrees = True)
                sd.putNumberArray("Translation " + cameraname,translation)
                sd.putNumberArray("Rotation " + cameraname,rotation)
                sd.putNumberArray("Reprojectionerr " + cameraname,reprojerr)
                sd.putNumber("Timestamp " + cameraname,timestamp)
                
            
            
        
        



class ATagCapture:
    
    #Constructor of AprilTagCapture
    def __init__(self, aprilTagSize, cameramtx_filename, cvSink, timediff, FRAME_WIDTH=640, FRAME_HEIGHT=480, families="tag36h11"):
        self.recSide = aprilTagSize
        
        #First set the video sink camera and properties
        self.cvSink = cvSink
        #self.cvSink = cv.VideoCapture(camera_number)
        
        #Load the camera matrix
        with np.load(cameramtx_filename) as X:
            self.mtx, self.dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]
        
        #allocate variable for images
        self.frame = np.zeros((FRAME_HEIGHT,FRAME_WIDTH,3), dtype=np.uint8)
        self.frame_grabtime = time.perf_counter()
        
        #Setup AprilTag properties
        #Create AprilTag Detector
        options = apriltag.DetectorOptions(families=families)
        self.detector = apriltag.Detector(options)
        self.results = []
        self.rvecs = []
        self.tvecs = []
        self.masks = []
        self.reprojectionerrors = []
        self.timediff = timediff
        
    def captureFrame(self):
        success, self.frame = self.cvSink.grabframe(self.frame)
        self.frame_grabtime = time.perf_counter()
        return success
    
    def genResults(self):
        gray = cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY)
        self.results = self.detector.detect(gray)
        unitsquare = np.array([[-1,1,0],[1,1,0],[1,-1,0],[-1,-1,0]])
        aprilSquare = self.recSide*unitsquare
        
        #Reset our translation and rotation vectors
        self.rvecs = []
        self.tvecs = []
        
        for tag in self.results:
            reorient = np.flip(tag.corners,0)
            mask, rvec, tvec, reprojerr = cv.solvePnP(aprilSquare, reorient,
                self.mtx, self.dist, flags = cv.SOLVEPNP_IPPE_SQUARE)
           #_,_,_,reprojerr = cv.solvePnPGeneric(aprilSquare, reorient,
            #    self.mtx, self.dist, flags = cv.SOLVEPNP_IPPE_SQUARE)
            
            #Append rotation and translation vectors to results
            self.rvecs.append(rvec[0])
            self.tvecs.append(tvec[0])
            self.masks.append(mask[0])
            self.reprojectionerrors.append(reprojerr)
            
        if len(self.results) > 0:
            return True
        
        return False
    
    def drawAxes(self):
        for (rvec,tvec) in zip(self.rvecs,self.tvecs):
            cv.drawFrameAxes(self.frame, self.mtx, self.dist, rvec, tvec, 3)
            
    def getTranslationsAngles(self,degrees = False):
        allEulerAngles = []
        
        for rvec in self.rvecs:
            to_euler = R.from_rotvec(rvec)
            allEulerAngles.append(to_euler.as_euler('yxz',degrees = degrees))
            
        return self.tvecs, allEulerAngles, self.reprojectionerrors, self.frame_grabtime+self.timediff
            
    def drawMask(self):
        coloring = [[[160]],[[160]],[[0]]]
        to_mask = np.repeat(np.repeat(coloring,3,axis=0),3,axis=1)
        for (number,mask) in zip(self.results.tag_id, self.masks):
            cv.bitwise_xor(self.frame, to_mask,self.frame,mask)
            
    def getFrame(self):
        return self.frame
    
    def getSink(self):
        return self.cvSink
        