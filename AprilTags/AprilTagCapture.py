# -*- coding: utf-8 -*-
"""
Created on Fri Oct 28 20:10:50 2022

@author: bayntuna
"""

import time

import cv2 as cv
import numpy as np
import pupil_apriltags
from cscore import CameraServer
from networktables import NetworkTables
from scipy.spatial.transform import Rotation as R
import glob


class NetworkTablesVisionHelper:
    
    
    
    
    #Constructor of Helper
    
        
    def __init__(self, ip_address, tablename = "SmartDashboard", aprilTagMaxNum = 10, FRAME_WIDTH=640, FRAME_HEIGHT=480):
        NetworkTables.initialize(ip_address)
        cs = CameraServer.getInstance()
        camerapaths =  sorted(glob.glob('/dev/v4l/by-path/*1.0-video-index0'))
        print(camerapaths)
        self.sd = NetworkTables.getTable(tablename)
        
        self.xTranslations = np.zeros(aprilTagMaxNum)
        self.yTranslations = np.zeros(aprilTagMaxNum)
        self.zTranslations = np.zeros(aprilTagMaxNum)
        self.yawRotations = np.zeros(aprilTagMaxNum)
        self.tagIDs = np.zeros(aprilTagMaxNum)
        self.confidences = np.zeros(aprilTagMaxNum)
        
        #Instantiate the usbcams and sink we're going to be using
        self.usbcams = []
        self.cvsinks = []
        self.outputStreams = []
        cameranames = ["Camera " + str(cameranum) for cameranum in range(len(camerapaths))]
        print(cameranames)
        self.apriltagcaptures = []
        self.timediff = 0
        
        #Add all cameras as dictated by initializer and start video feed of them
        for cameraname, camerapath in zip(cameranames,camerapaths):
            print(cameraname)
            try:
                self.usbcams.append(cs.startAutomaticCapture(name=cameraname, path = camerapath))
                self.usbcams[-1].setResolution(FRAME_WIDTH,FRAME_HEIGHT)
                self.cvsinks.append(cs.getVideo(name = cameraname))
                print(camerapath)   
                self.outputStreams.append(cs.putVideo(str(cameraname)+" modified",FRAME_WIDTH,FRAME_HEIGHT))
            except:
                print("No camera at " + camerapath)
        
        
        
        #Startup the networktable
        self.sd.putStringArray("CameraNames", cameranames)

    
    def getSinks(self):
        return self.cvsinks
        
    def getFrames(self):
        return [apriltagcapture.getFrame() for apriltagcapture in self.apriltagcaptures]
    
    def syncTimes(self,robottime="RobotTime"):
        self.timediff = self.sd.getNumber(robottime, 0) - time.perf_counter()
        
        
    def addAprilTagCapturing(self, atagcapturing):
        self.apriltagcaptures.append(atagcapturing)
        

    
    def processVideos(self, drawAxes = False, drawMask = False):
        for atagcapture in self.apriltagcaptures:
            if atagcapture.captureFrame() > 0 and atagcapture.genResults():
                if drawAxes: atagcapture.drawAxes() 
                if drawMask: atagcapture.drawMask() 
                cameraname = atagcapture.getSink().getSource().getName()
                cameratable = self.sd.getSubTable(cameraname)
                translations, rotations, reprojerrors, timestamp, seenTagIDs = atagcapture.getTranslationsAngles(degrees = True)
                self.tagIDs = 0*self.tagIDs
                for (tagID,translation,rotation,reprojerror) in zip(seenTagIDs,translations,rotations,reprojerrors):
                    self.xTranslations[tagID] = translation[0]
                    self.yTranslations[tagID] = translation[1]
                    self.zTranslations[tagID] = translation[2]
                    self.yawRotations[tagID] = rotation[0]
                    self.confidences[tagID] = self.calculateConfidence(reprojerror)
                    self.tagIDs[tagID] = 1
                
                
                #Put Data into networktables
                cameratable.putNumberArray("xTranslation",self.xTranslations)
                cameratable.putNumberArray("yTranslation",self.yTranslations)
                cameratable.putNumberArray("zTranslation",self.zTranslations)
                cameratable.putNumberArray("yawRotation",self.yawRotations)
                cameratable.putNumberArray("Confidence",self.confidences)
                cameratable.putNumber("Timestamp",timestamp+self.timediff)
                cameratable.putNumberArray("AprilTagIDs",seenTagIDs)
                
            
    def outputVideo(self):
        for (atagcapture, outputStream) in zip(self.apriltagcaptures,self.outputStreams):
            print(outputStream)
            outputStream.putFrame(atagcapture.getFrame())
        
    def calculateConfidence(self, reprojectionerrors):
        return ((reprojectionerrors[1]-reprojectionerrors[0])/reprojectionerrors[1] if reprojectionerrors[1]>0 else 0)

            
        
        



class ATagCapture:
    
    #Constructor of AprilTagCapture
    def __init__(self, aprilTagSize, cameramtx_filename, cvSink, FRAME_WIDTH=640, FRAME_HEIGHT=480, families="tag36h11"):
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
        
        self.detector = pupil_apriltags.Detector(families = families, nthreads = 4, quad_decimate = 1.0, quad_sigma=1.0)
        self.results = []
        self.rvecs = []
        self.tvecs = []
        self.masks = []
        self.reprojectionerrors = []
        self.tagids = []
        
    def captureFrame(self):
        success, self.frame = self.cvSink.grabFrame(self.frame)
        self.frame_grabtime = time.perf_counter()
        return success
    
    def genResults(self):
        gray = cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY)
        self.results = self.detector.detect(gray)
        unitsquare = np.array([[-1,1,0],[1,1,0],[1,-1,0],[-1,-1,0]])
        aprilSquare = self.recSide*0.5*unitsquare
        
        #Reset our translation and rotation vectors
        self.tagids = []
        self.rvecs = []
        self.tvecs = []
        self.masks = []
        self.reprojectionerrors = []
        
        for tag in self.results:
            #reorient = np.flip(tag.corners,0)
            reorient = tag.corners
            reprojerr = 0
            # _, rvec, tvec = cv.solvePnP(aprilSquare, reorient,
            #                                                  self.mtx, self.dist, flags = cv.SOLVEPNP_IPPE_SQUARE)
            _, rvec, tvec,reprojerr = cv.solvePnPGeneric(aprilSquare, reorient,
                                                self.mtx, self.dist, flags = cv.SOLVEPNP_IPPE_SQUARE)
            
            #Append rotation and translation vectors to results
            mask = np.zeros_like(self.frame)
            mask = cv.fillPoly(mask, np.int32([reorient]), color=(0,128,0))
            
            self.tagids.append(tag.tag_id)
            self.rvecs.append(rvec[0])
            self.tvecs.append(tvec[0])
            self.masks.append(mask)
            self.reprojectionerrors.append(reprojerr[:,0])
            
        if len(self.results) > 0:
            return True
        
        return False
    
    def drawAxes(self):
        for (rvec,tvec) in zip(self.rvecs,self.tvecs):
            cv.drawFrameAxes(self.frame, self.mtx, self.dist, rvec, tvec, self.recSide)
            
    def getTranslationsAngles(self,degrees = False):
        allEulerAngles = []
        
        for rvec in self.rvecs:
            to_euler = R.from_rotvec(rvec.reshape(3,))
            allEulerAngles.append(to_euler.as_euler('yxz',degrees = degrees))
        
        print(allEulerAngles)
        if len(allEulerAngles)>0 and np.abs(allEulerAngles[0][2])>90:
            self.tvecs[:][0:1] = (-np.concatenate(self.tvecs,axis=0)[0:1,:]).transpose().tolist()
            
        return self.tvecs, allEulerAngles, self.reprojectionerrors, self.frame_grabtime, self.tagids
            
    def drawMask(self):
        results_id = [result.tag_id for result in self.results]
        for (number,mask) in zip(results_id, self.masks):
            self.frame = cv.bitwise_xor(self.frame, mask)
            
    def getFrame(self):
        return self.frame
    
    def getSink(self):
        return self.cvSink
        