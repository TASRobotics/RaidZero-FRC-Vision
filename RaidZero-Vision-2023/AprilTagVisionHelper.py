import time

import cv2 as cv
import numpy as np
import pupil_apriltags
from cscore import CameraServer
from networktables import NetworkTables
from scipy.spatial.transform import Rotation as R
import glob

class AprilTagVisionHelper:
    def __init__(self, ip_address, cone_camera_path, tablename = "SmartDashboard", aprilTagMaxNum = 9, FRAME_WIDTH=640, FRAME_HEIGHT=480):
        NetworkTables.initialize(ip_address)
        cs = CameraServer.getInstance()
        camerapaths =  sorted(glob.glob('/dev/v4l/by-path/*1.0-video-index0'))
        try:
            camerapaths.remove(cone_camera_path)
        except:
            pass
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