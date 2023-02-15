# -*- coding: utf-8 -*-
"""
Created on Sun Feb 12 17:29:54 2023

@author: bayntuna
"""

numpoints = 20
from sklearn.linear_model import LogisticRegression
import numpy as np

myModel = LogisticRegression()
xData = np.transpose(np.array([range(numpoints)])-15)
# print(xData)
myData = np.append(xData,xData**2,axis=1)
# print(myData)
myValues = np.zeros([numpoints])
myValues[:5]=1
# myValues[0]=1
# myValues[1]=1
# myValues[2]=1

# myValues[20]=1
# myValues[31]=1
# myValues[52]=1
myModel.fit(myData, myValues)
Theta0 = myModel.intercept_
Theta1 = myModel.coef_
coeffs = np.flip(myModel.coef_[0])
coeffs = np.append(coeffs,myModel.intercept_)
edges = np.roots(coeffs)
meanPosition = np.mean(myData,0)[0]
closestEdge = np.abs(edges-meanPosition).argmin()
edgeHigher = np.sign(closestEdge-meanPosition)
isFalling = myModel.decision_function(np.array([closestEdge-edgeHigher,(closestEdge-edgeHigher)**2]).reshape(1,-1))>0
