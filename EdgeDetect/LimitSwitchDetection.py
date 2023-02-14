# -*- coding: utf-8 -*-
"""
Created on Sat Feb 11 16:05:14 2023

@author: bayntuna
"""

from sklearn.linear_model import LogisticRegression
import numpy as np
from networktables import NetworkTables

class WristAlignment:
    
    def __init__(self, table, bufferSize = 150, 
                 incomingKey = "LimitSwitchData", outgoingKey = "EdgeData"):
        self.bufferPosition = 0
        self.limitSwitchModel = LogisticRegression()
        self.dataTable = table
        self.incomingKey = incomingKey
        self.outgoingKey = outgoingKey
        self.encoderData = np.ones(bufferSize,2)*(-1000)
        self.limitData = np.zeros(bufferSize)
        self.bufferFilled = False
        
    def update(self):
        incomingData = self.dataTable.getNumberArray(self.incomingKey, -1000)
        self.encoderData[self.bufferPosition] = [incomingData[0],incomingData[0]**2]
        self.limitData[self.bufferPosition] = incomingData[1]
        self.bufferPosition += 1
        self.bufferFilled = True if self.bufferPosition>len(self.limitData()) else self.bufferFilled
        self.bufferPosition %=len(self.limitData)
        
        if self.bufferFilled and (np.mean(self.limitData)-0.5)**2<0.2:
            self.limitSwitchModel.fit(self.encoderData(), self.limitData)
            self.dataTable.putNumberArray(self.outgoingKey, findEdges)
            
            
    def findEdges(self):
        coeffs = np.flip(self.limitSwitchModel.coef_[0])
        coeffs = np.append(coeffs,self.limitSwitchModel.intercept_)
        edges = np.roots(coeffs)
        meanPosition = np.mean(self.encoderData,0)[0]
        closestEdge = np.abs(edges-meanPosition).argmin()
        edgeHigher = np.sign(closestEdge-meanPosition)
        isFalling = self.limitSwitchModel.decision_function(np.array([closestEdge-edgeHigher,(closestEdge-edgeHigher)**2]).reshape(1,-1))
        return [closestEdge, isFalling]
            
        

>>> clf.predict(X[:2, :])