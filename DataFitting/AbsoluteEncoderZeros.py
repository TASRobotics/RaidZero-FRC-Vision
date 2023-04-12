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
    
    def __init__ (self, ip_address, table, encoderNames = ["Proximal", "Distal"], bufferSize = 50, 
                 incomingKey = "Angle", outgoingKey = "Offset"):
        NetworkTables.initialize(ip_address)
        self.bufferPosition = 0
        self.bufferSize = bufferSize
        self.dataTable = NetworkTables.getTable(table)
        self.incomingKey = incomingKey
        self.outgoingKey = outgoingKey
        self.absEncoderModel = []
        self.relEncoderData = []
        self.absEncoderData = []
        self.encoderNames = encoderNames
        for modelName in range (encoderNames):
            self.absEncoderModel.append(LinearRegression())
            self.relEncoderData.append(np.zeros([bufferSize,1]))
            self.absEncoderData.append(np.zeros([bufferSize,1]))
        
        self.cycleTime = time.perf_counter()
        
    def run(self):
        if time.perf_counter() - self.cycleTime>0.1:
            self.cycleTime = time.perf_counter()
            # defaultData = [0]*2*len(self.absEncoderModel)
            defaultData = 0
            intercepts = [0]*len(self.absEncoderModel)
            
            for absencoder,relencoder,model,name in zip(self.absEncoderData,self.relEncoderData,self.absEncoderModel,self.encoderNames):
                
                relencoder[self.bufferPosition] = self.dataTable.getNumber(name+ " " + self.incomingKey, defaultData)
                absencoder[self.bufferPosition] = self.dataTable.getNumber(name+" Absolute " +self.incomingKey, defaultData)
                if self.bufferPosition == self.bufferSize - 1:
        
                    model.fit(absencoder,relencoder)
                    
            
            intercepts = [model.intercept_ for model in self.absEncoderModel]
            if np.isreal(intercepts).all():
                self.dataTable.putNumberArray(self.outgoingKey, intercepts)

            
            self.bufferPosition += 1    
            self.bufferPosition %=self.bufferSize
            # print(self.bufferFilled)
            # print(np.mean(self.limitData)-0.5)
            