# -*- coding: utf-8 -*-
"""
Created on Wed Feb 22 10:15:08 2023

@author: bayntuna
"""

from sklearn.linear_model import LinearRegression
import numpy as np
from networktables import NetworkTables
import time

class CalculateZeros:
    
    def __init__ (self, ip_address, table, numFits = 2, bufferSize = 50, 
                 incomingKey = "EncoderData", outgoingKey = "Offset"):
        NetworkTables.initialize(ip_address)
        self.bufferPosition = 0
        self.bufferSize = bufferSize
        self.dataTable = NetworkTables.getTable(table).getSubTable("Arm")
        self.incomingKey = incomingKey
        self.outgoingKey = outgoingKey
        self.absEncoderModel = []
        self.relEncoderData = []
        self.absEncoderData = []
        for modelNum in range (numFits):
            self.absEncoderModel.append(LinearRegression())
            self.relEncoderData.append(np.zeros([bufferSize,1]))
            self.absEncoderData.append(np.zeros([bufferSize,1]))
        
        self.cycleTime = time.perf_counter()
        
    def run(self):
        if time.perf_counter() - self.cycleTime>0.1:
            self.cycleTime = time.perf_counter()
            self.bufferPosition += 1
            defaultData = [0]*2*len(self.absEncoderModel)
            intercepts = [0]*len(self.absEncoderModel)
            
            for encoderNum in range(len(self.absEncoderModel)):
                
                incomingData = self.dataTable.getNumberArray(self.incomingKey+str(encoderNum), defaultData)
                self.absEncoderData[encoderNum][self.bufferPosition] = incomingData[encoderNum*2]
                self.relEncoderData[encoderNum][self.bufferPosition] = incomingData[encoderNum*2+1]
                self.absEncoderModel[encoderNum]
                if self.bufferPosition == self.bufferSize:
        
                    self.absEncoderModel[encoderNum].fit(
                        self.absEncoderData[encoderNum], 
                        self.relEncoderData[encoderNum])
                    
                    intercepts[encoderNum] = self.absEncoderModel[encoderNum].intercept_
                    
            if np.isreal(intercepts).all():
                self.dataTable.putNumberArray(self.outgoingKey, intercepts)

                
            self.bufferPosition %=len(self.bufferSize)
            # print(self.bufferFilled)
            # print(np.mean(self.limitData)-0.5)
            