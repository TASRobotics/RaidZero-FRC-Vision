import time
import cv2 as cv
import numpy as np

class ConeCapture:
    def __init__(self, cv_sink, yellow_lower, yellow_upper, frame_width, frame_height, area_threshold):
        self.cv_sink = cv_sink

        self.frame = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
        self.hsv_frame = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
        self.mask = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
        self.yellow_threshold = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)

        self.frame_grab_time = time.perf_counter()

        self.x_translation, self.y_translation = 0, 0

        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0

        self.point_x, self.point_y = 0, 0

        self.frame_width = frame_width
        self.frame_height = frame_height

        self.area_threshold = area_threshold

        self.yellow_lower = np.array(yellow_lower, dtype='uint8')
        self.yellow_upper = np.array(yellow_upper, dtype='uint8')

        self.yellow_contours = []

    def processFrame(self):
        self.hsv_frame = cv.cvtColor(self.frame, cv.COLOR_BGR2HSV)

        self.yellow_threshold = cv.inRange(self.hsv_frame, self.yellow_lower, self.yellow_upper)
        (contours, _) = cv.findContours(self.yellow_threshold, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        self.yellow_contours = []
        merge_contour = []

        for contour in contours:
            contour_area = cv.contourArea(contour)
            if contour_area > self.area_threshold:
                merge_contour.extend(contour)
                self.yellow_contours.append(contour)

        merge_contour_x = np.array(list(map(lambda point: point.tolist()[0][0], merge_contour)))
        merge_contour_y = np.array(list(map(lambda point: point.tolist()[0][1], merge_contour)))

        if len(merge_contour) > 0:
            self.min_x, self.min_y = np.min(merge_contour_x), np.min(merge_contour_y)
            self.max_x, self.max_y = np.max(merge_contour_x), np.max(merge_contour_y)

            self.point_x = (self.min_x + self.max_x) / 2
            self.point_y = (self.min_y + self.max_y) / 2

            self.x_translation = int(self.point_x - self.frame_width / 2)
            self.y_translation = -int(self.point_y - self.frame_height / 2)

            return True
        else:
            self.x_translation, self.y_translation = 0, 0

        return False

    def captureFrame(self):
        success, self.frame = self.cv_sink.grabFrame(self.frame)
        self.frame_grab_time = time.perf_counter()
        return success

    def drawMask(self):
        self.frame = cv.drawContours(np.zeros(self.frame.shape, np.uint8), tuple(self.yellow_contours), -1, cv.mean(self.frame, mask=self.yellow_threshold), -1)

    def drawMarker(self):
        cv.drawMarker(self.frame, (self.point_x, self.point_y), (255, 255, 255), cv.MARKER_CROSS, 50, 3)

    def drawRectangle(self):
        cv.rectangle(self.frame, (self.min_x, self.min_y), (self.max_x, self.max_y), (255, 255, 255), 2)

    def getFrame(self):
        return self.frame

    def getCvSink(self):
        return self.cv_sink

    def getXTranslation(self):
        return self.x_translation

    def getYTranslation(self):
        return self.y_translation
