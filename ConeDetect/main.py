import cv2 as cv
import time

from networktables import NetworkTables
from VisionHelper import VisionHelper

table_name = "SmartDashboard"
# ip_address = "10.6.21.133"
# ip_address = "192.168.1.163"
# ip_address = "192.11.22.1"
ip_address = "10.42.53.2"
camera_paths = "/dev/v4l/by-path/*1.0-video-index0"

NetworkTables.initialize()
network_table = NetworkTables.getTable(table_name)

frame_width = 640
frame_height = 480
area_threshold = 20000

yellow_lower = [0, 56, 152]
yellow_upper = [76, 238, 255]

local_clock = 0
remote_clock = 0

def syncNetworkTableTime(table, key, value):
    global local_clock
    global remote_clock
    if key == "robot_time":
        local_clock = time.perf_counter()
        remote_clock = value
        network_table.putNumber("RPiTime", value)

if __name__ == '__main__':
    vision_helper = VisionHelper(ip_address, camera_paths, yellow_lower, yellow_upper, frame_width=frame_width, frame_height=frame_height, area_threshold=area_threshold)
    network_table.addEntryListener(syncNetworkTableTime, key="robot_time", immediateNotify=True)
    vision_helper.syncTimes()
    vision_helper.setupCamera()
    while True:
        try:
            vision_helper.processVideo()
            vision_helper.outputVideo()
        except:
            vision_helper.setupCamera()

        key = cv.waitKey(1) & 0xFF
        if key == ord("q"):
            break

cv.destroyAllWindows()
