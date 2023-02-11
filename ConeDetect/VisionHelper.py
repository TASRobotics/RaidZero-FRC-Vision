import time
import glob

from cscore import CameraServer
from networktables import NetworkTables
from ConeCapture import ConeCapture

class VisionHelper:
    def __init__(self, ip_address, camera_paths, yellow_lower, yellow_upper, table_name="SmartDashboard", camera_name="Cone Camera", frame_width=640, frame_height=480, area_threshold=2000):
        NetworkTables.initialize(ip_address)

        self.camera_server = CameraServer.getInstance()
        self.network_table = NetworkTables.getTable(table_name)

        self.camera_paths = camera_paths

        self.camera_name = camera_name

        self.usb_cams = None
        self.output_stream = None
        self.cone_capture = None

        self.frame_width = frame_width
        self.frame_height = frame_height
        self.area_threshold = area_threshold

        self.yellow_lower = yellow_lower
        self.yellow_upper = yellow_upper

        self.time_diff = 0

        self.network_table.putString("Camera Name", self.camera_name)

    def setupCamera(self):
        try:
            self.camera_path = sorted(glob.glob(self.camera_paths))[0]
            self.usb_cam = self.camera_server.startAutomaticCapture(name=self.camera_name, path=self.camera_path)
            self.usb_cam.setResolution(self.frame_width, self.frame_height)
            self.cv_sink = self.camera_server.getVideo(name=self.camera_name)
            self.cone_capture = ConeCapture(self.cv_sink, self.yellow_lower, self.yellow_upper, self.frame_width, self.frame_height, self.area_threshold)
            self.output_stream = self.camera_server.putVideo("Output Stream", self.frame_width, self.frame_height)
        except:
            print("Failed to setup camera.")

    def processVideo(self, draw_mask = False, draw_marker=False, draw_rectangle=False):
        if self.cone_capture.captureFrame() > 0:
            if self.cone_capture.processFrame():
                if draw_mask:
                    self.cone_capture.drawMask()
                if draw_marker:
                    self.cone_capture.drawMarker()
                if draw_rectangle:
                    self.cone_capture.drawRectangle()
                camera_table = self.network_table.getSubTable(self.camera_name)
                camera_table.putNumber("X Translation", self.cone_capture.getXTranslation())
                camera_table.putNumber("Y Translation", self.cone_capture.getYTranslation())
                print(self.cone_capture.getXTranslation(), self.cone_capture.getYTranslation())
            else:
                print("No Cone Detected.")

    def outputVideo(self):
        self.output_stream.putFrame(self.cone_capture.getFrame())

    def syncTimes(self, robot_time="RobotTime"):
        self.time_diff = self.network_table.getNumber(robot_time, 0) - time.perf_counter()

    def getConeCapture(self):
        return self.cone_capture

    def getFrame(self):
        return self.cone_capture.getFrame()

    def getSink(self):
        return self.cv_sink

    def getCameraPathExist(self):
        return len(self.camera_paths) > 0
