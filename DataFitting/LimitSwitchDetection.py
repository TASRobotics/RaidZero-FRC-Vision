# -*- coding: utf-8 -*-
"""
Created on Sat Feb 11 16:05:14 2023

@author: bayntuna
"""

from sklearn.linear_model import LogisticRegression
import numpy as np
from networktables import NetworkTables
import time

class WristAlignment:
    
    def __init__(self, ip_address, table, bufferSize = 50, 
                 incomingKey = "LimitSwitchData", outgoingKey = "EdgeData"):
        NetworkTables.initialize(ip_address)
        self.bufferPosition = 0
        self.limitSwitchModel = LogisticRegression()
        self.dataTable = NetworkTables.getTable(table).getSubTable("Wrist")
        self.incomingKey = incomingKey
        self.outgoingKey = outgoingKey
        self.encoderData = np.ones([bufferSize,2])*(-1000)
        self.limitData = np.zeros(bufferSize)
        self.bufferFilled = False
        self.cycleTime = time.perf_counter()
        
    def run(self):
        if time.perf_counter() - self.cycleTime>0.1:
            self.cycleTime = time.perf_counter()
            incomingData = self.dataTable.getNumberArray(self.incomingKey, [-1000,0])
            if incomingData[0]==-1000: return
            self.encoderData[self.bufferPosition] = [incomingData[1],incomingData[1]**2]
            self.limitData[self.bufferPosition] = incomingData[0]
            self.bufferPosition += 1
            # print(self.bufferPosition)
            if self.bufferPosition==len(self.limitData):  self.bufferFilled = True
            self.bufferPosition %=len(self.limitData)
            # print(self.bufferFilled)
            # print(np.mean(self.limitData)-0.5)
            if self.bufferFilled and (np.mean(self.limitData)-0.5)**2<0.2:
    
                self.limitSwitchModel.fit(self.encoderData, self.limitData)
                
                coeffs = np.flip(self.limitSwitchModel.coef_[0])
                coeffs = np.append(coeffs,self.limitSwitchModel.intercept_)
                edges = np.roots(coeffs)
                
                if np.isreal(edges).all():
                    self.dataTable.putNumberArray(self.outgoingKey, self.findEdges())
                    self.bufferFilled = False
                    self.bufferPosition = 0
                    print("Calculating edge")
            
            
    def findEdges(self, edges):
        meanPosition = np.mean(self.encoderData,0)[0]
        closestEdge = edges[np.abs(edges-meanPosition).argmin()]
        fromEdge = np.abs((closestEdge-meanPosition)/10.0)
        isFalling = self.limitSwitchModel.decision_function(np.array([closestEdge-fromEdge,(closestEdge-fromEdge)**2]).reshape(1,-1))
        return [closestEdge, isFalling]
            
        
