# -*- coding: utf-8 -*-
"""
Created on Fri Oct 28 20:10:50 2022

@author: bayntuna
"""

import time

import cv2 as cv
import numpy as np
import pupil_apriltags
# from cscore import CameraServer
# from networktables import NetworkTables
from scipy.spatial.transform import Rotation as R
# import glob



class ATagCapture:
    
    
    #Constructor of AprilTagCapture
    def __init__(self, aprilTagSize, cameramtx_filename, cvSink, FRAME_WIDTH=640, FRAME_HEIGHT=480, families="tag16h5", aprilTagMaxNum = 8):
        self.recSide = aprilTagSize
        
        #First set the video sink camera and properties
        self.cvSink = cvSink
        #self.cvSink = cv.VideoCapture(camera_number)
        
        #Load the camera matrix
        with np.load(cameramtx_filename) as X:
            print("Loading matrix: "+str(cameramtx_filename))
            self.mtx, self.dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]
        
        #allocate variable for images
        self.frame = np.zeros((FRAME_HEIGHT,FRAME_WIDTH,3), dtype=np.uint8)
        self.frame_grabtime = time.perf_counter()
        
        #Setup AprilTag properties
        #Create AprilTag Detector
        
        self.detector = pupil_apriltags.Detector(families = families, nthreads = 4, quad_decimate = 2.0, quad_sigma=1.6)
        self.aprilTagMaxNum = aprilTagMaxNum
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
            
            if (tag.hamming==0 and tag.tag_id <= self.aprilTagMaxNum): 
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
        
        # print("Printing IDs")
        # print(self.tagids)
        # print("Printing Angles")
        # print(allEulerAngles)
        # print("Printing Translations")
        # print(self.tvecs)
        # Checking if camera is rotated by orientation of Apriltag
        camera_rotation_correction = np.eye(3)
        
        
        if len(allEulerAngles)>0:
            corrected_tvec = np.matrix(np.concatenate(np.array(self.tvecs),axis=1))
            
            # print("Corrected tvec")
            # print(corrected_tvec)
            
            if np.abs(allEulerAngles[0][2])>135:
                camera_rotation_correction = np.matrix([[-1,0,0],[0,-1,0],[0,0,1]])
                
            elif allEulerAngles[0][2]>45 and allEulerAngles[0][2]<=135:
                camera_rotation_correction = np.matrix([[0,1,0],[-1,0,0],[0,0,1]])
                
            elif allEulerAngles[0][2]<-45 and allEulerAngles[0][2]>=-135:
                camera_rotation_correction = np.matrix([[0,-1,0],[1,0,0],[0,0,1]])
                
            corrected_tvec = camera_rotation_correction*corrected_tvec
            self.tvecs = list(np.expand_dims(np.array(corrected_tvec.transpose()),axis=2))
        # print("New Corrected tvec")
        # print(self.tvecs)
        # self.tvecs[:][0:1] = (-np.concatenate(self.tvecs,axis=0)[0:1,:]).transpose().tolist()
            
        return self.tvecs, allEulerAngles, self.reprojectionerrors, self.frame_grabtime, self.tagids
            
    def drawMask(self):
        results_id = [result.tag_id for result in self.results]
        for (number,mask) in zip(results_id, self.masks):
            self.frame = cv.bitwise_xor(self.frame, mask)
            
    def getFrame(self):
        return self.frame
    
    def getSink(self):
        return self.cvSink
        