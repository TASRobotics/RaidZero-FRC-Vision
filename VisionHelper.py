import time
import glob

from cscore import CameraServer
from networktables import NetworkTables
from ConeDetect.ConeCapture import ConeCapture
import numpy as np
# import cv2 as cv
from AprilTags.AprilTagCapture import ATagCapture

class NetworkTablesVisionHelper:


    aprilTagCameras = 0
    coneDetectionCameras = 0

    #Constructor of Helper


    def __init__(self, ip_address,tablename, FRAME_WIDTH=640, FRAME_HEIGHT=480):
        NetworkTables.initialize(ip_address)
        cs = CameraServer.getInstance()
        camerapaths =  sorted(glob.glob('/dev/v4l/by-path/*1.0-video-index0'))
        print(camerapaths)
        self.sd = NetworkTables.getTable(tablename)

        #Instantiate the usbcams and sink we're going to be using
        self.frame_width, self.frame_height = FRAME_WIDTH, FRAME_HEIGHT
        self.usbcams = []
        self.cvsinks = []
        self.outputStreams = []
        cameranames = ["Camera " + str(cameranum) for cameranum in range(len(camerapaths))]
        print(cameranames)
        self.apriltagcaptures = []
        self.robotCaptureTime = 0
        self.startTime = time.perf_counter()

        #Add all cameras as dictated by initializer and start video feed of them
        for cameraname, camerapath in zip(cameranames,camerapaths):
            print(cameraname)
            try:
                print("Initializing Camera " + str(cameraname))
                self.usbcams.append(cs.startAutomaticCapture(name=cameraname, path = camerapath))
                self.usbcams[-1].setResolution(FRAME_WIDTH,FRAME_HEIGHT)
                self.cvsinks.append(cs.getVideo(name = cameraname))
                print(camerapath)
                self.outputStreams.append(cs.putVideo(str(cameraname)+" modified",FRAME_WIDTH,FRAME_HEIGHT))
                print(len(self.cvsinks))
            except:
                print("No camera at " + camerapath)



        #Startup the networktable
        self.sd.putStringArray("CameraNames", cameranames)

    def initializeDetectors(self, numColourCameras, aTagSize, cameramtx_path,
                            area_threshold, yellow_lower, yellow_upper, aprilTagMaxNum = 9):
        self.coneDetectionCameras = numColourCameras
        self.initializeAprilTagDetect(aTagSize, cameramtx_path, aprilTagMaxNum)
        self.initializeConeDetect(area_threshold, yellow_lower, yellow_upper)


    def initializeAprilTagDetect(self, aTagSize,cameramtx_path,aprilTagMaxNum = 9):

        self.xTranslations = np.zeros(aprilTagMaxNum)
        self.yTranslations = np.zeros(aprilTagMaxNum)
        self.zTranslations = np.zeros(aprilTagMaxNum)
        self.yawRotations = np.zeros(aprilTagMaxNum)
        self.tagIDs = np.zeros(aprilTagMaxNum)
        self.confidences = np.zeros(aprilTagMaxNum)

        matrices = sorted(glob.glob(cameramtx_path+'*mtx.npz'))
        print("Using the following camera matrices:")
        print(matrices)
        for cvSink,mtx in zip(self.cvsinks[:-1],matrices):
            print("Adding Apriltag captures "+ str(mtx))
            self.apriltagcaptures.append(ATagCapture(aTagSize,mtx,cvSink))
            print("April Tag Capture Length: ", len(self.apriltagcaptures))

        self.aprilTagCameras = len(self.apriltagcaptures)

    def initializeConeDetect(self, area_threshold, yellow_lower, yellow_upper):
        # self.coneDetectionCameras = 1
        for cvSink in self.cvsinks[self.aprilTagCameras:]:

            self.cone_capture = ConeCapture(cvSink, yellow_lower, yellow_upper, self.frame_width, self.frame_height, area_threshold)

        self.aprilTagCameras = len(self.apriltagcaptures)

        self.x_translation, self.y_translation = 0, 0
        self.point_x, self.point_y = 0, 0
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0

        self.area_threshold = area_threshold

        self.yellow_lower = yellow_lower
        self.yellow_upper = yellow_upper


    def getSinks(self):
        return self.cvsinks

    def getFrames(self):
        frames = [apriltagcapture.getFrame() for apriltagcapture in self.apriltagcaptures]
        frames.insert(0,self.cone_capture.getFrame())
        return frames

    def syncTimes(self,robottime="RobotTime"):
        self.robotCaptureTime = self.sd.getNumber(robottime, 0)


    # def addAprilTagCapturing(self, atagcapturing):
    #     self.apriltagcaptures.append(AprilTagCapture())

    def processVideos(self, drawAxes=False, drawMask=False, drawMarker=False, drawRectangle=False):

        self.processAprilTagVideos(drawAxes, drawMask)
        # self.processConeVideo(draw_mask=drawMask, draw_marker=drawMarker, draw_rectangle=drawRectangle)

    def processAprilTagVideos(self, drawAxes, drawMask):

        for atagcapture in self.apriltagcaptures:
            self.syncTimes()
            if atagcapture.captureFrame() > 0 and atagcapture.genResults():
                if drawAxes: atagcapture.drawAxes()
                if drawMask: atagcapture.drawMask()
                cameraname = atagcapture.getSink().getSource().getName()
                cameratable = self.sd.getSubTable(cameraname)
                # print("Camera Table: " + str(cameratable))
                translations, rotations, reprojerrors, timestamp, seenTagIDs = atagcapture.getTranslationsAngles(degrees = True)

                # Clear arrays
                self.tagIDs = 0*self.tagIDs
                self.xTranslations = 0*self.xTranslations
                self.yTranslations = 0*self.yTranslations
                self.zTranslations = 0*self.zTranslations
                self.yawRotations = 0*self.yawRotations
                self.confidences = 0*self.confidences

                for (tagID,translation,rotation,reprojerror) in zip(seenTagIDs,translations,rotations,reprojerrors):
                    self.xTranslations[tagID] = translation[0]
                    self.yTranslations[tagID] = translation[1]
                    self.zTranslations[tagID] = translation[2]
                    self.yawRotations[tagID] = rotation[0]
                    self.confidences[tagID] = self.calculateConfidence(reprojerror)
                    self.tagIDs[tagID] = 1

                self.syncTimes()
                #Put Data into networktables
                cameratable.putNumberArray("xTranslation",self.xTranslations)
                cameratable.putNumberArray("yTranslation",self.yTranslations)
                cameratable.putNumberArray("zTranslation",self.zTranslations)
                cameratable.putNumberArray("yawRotation",self.yawRotations)
                cameratable.putNumberArray("Confidence",self.confidences)
                cameratable.putNumber("Timestamp",self.robotCaptureTime)
                cameratable.putNumberArray("AprilTagIDs",seenTagIDs)

    def processConeVideo(self, draw_mask = False, draw_marker=False, draw_rectangle=False):
        if self.cone_capture.captureFrame() > 0:
            if self.cone_capture.processFrame():
                if draw_mask:
                    self.cone_capture.drawMask()
                if draw_marker:
                    self.cone_capture.drawMarker()
                if draw_rectangle:
                    self.cone_capture.drawRectangle()
            camera_table = self.sd.getSubTable(self.cone_capture.getCvSink().getSource().getName())
            camera_table.putNumber("X Translation", self.cone_capture.getXTranslation())
            camera_table.putNumber("Y Translation", self.cone_capture.getYTranslation())
                # print(self.cone_capture.getXTranslation(), self.cone_capture.getYTranslation())
            # else:
                # print("No Cone Detected.")


    def outputVideo(self):
        for (atagcapture, outputStream) in zip(self.apriltagcaptures,self.outputStreams[:-1]):
            #print(outputStream)
            outputStream.putFrame(atagcapture.getFrame())

        for outputStream in self.outputStreams[-1:]:
            #print(outputStream)
            outputStream.putFrame(self.cone_capture.getFrame())

    def calculateConfidence(self, reprojectionerrors):
        return ((reprojectionerrors[1]-reprojectionerrors[0])/reprojectionerrors[1] if reprojectionerrors[1]>0 else 0)
