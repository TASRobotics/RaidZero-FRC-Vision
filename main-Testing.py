# -*- coding: utf-8 -*-
"""
Created on Tue Oct 11 18:48:53 2022

@author: bayntuna
"""

import time
from networktables import NetworkTables
from scipy.spatial.transform import Rotation as R

import logging 


finished = False
j = R.from_rotvec([0.,0.,0.])

NetworkTables.initialize()

sd = NetworkTables.getTable("SmartDashboard")

def send_timestamp():
    tosend = time.perf_counter()
    print("Send time:    " + str(tosend))
    sd.putNumber("RobotTime", tosend)

def get_timestamp():
    now = time.perf_counter()
    print("Receive time: " + str(sd.getNumber("RPitime", -1)))
    print("Now: " + str(now))


def increase_angle():
    global j
    global finished
    i = 0.
    while i<50000:
        i += 1
        #time.sleep(.01)
        #print(i)
        j=R.from_rotvec([0,0,i/1000.])
    finished = True
        
def convert_angle():
    global finished
    while not finished:
        temp=j.as_euler('xyz',degrees=True)
        temp=j.as_matrix()
        temp = j.as_quat()
        print(temp)
            #print("hi")
            #print(str(j.as_euler('xyz',degrees=True)))
            
def acquire_pose(table, key, value, isnew):
    print("hi")

if __name__ == '__main__':
    

    # th = Process(target = increase_angle)
    # th2 = Process(target = convert_angle)
    start = time.perf_counter()
    # th.start()
    # th2.start()
    
    # th.join()
    # th2.join()

    
    logging.basicConfig(level=logging.DEBUG)
    

   #sd.addEntryListener("Translation", acquire_pose,flags = 0x06)
    #sd.addEntryListenerEx(listener = acquire_pose, key = "Translation", immediateNotify=True)
    
    print(time.perf_counter()-start)
    start = time.perf_counter()
    
    i=0
    while True:
        send_timestamp()
        time.sleep(.08)
        get_timestamp()
        time.sleep(1)
        
        
    print(time.perf_counter()-start)