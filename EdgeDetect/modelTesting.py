# -*- coding: utf-8 -*-
"""
Created on Sun Feb 12 17:29:54 2023

@author: bayntuna
"""


from sklearn.linear_model import LogisticRegression
import numpy as np

myModel = LogisticRegression()
myData = np.transpose(np.array([range(180),np.array(range(180))**2]))
myValues = np.zeros([180])
myValues[160:]=1
myValues[0]=1
myValues[1]=1
myValues[2]=1

myValues[20]=1
myValues[31]=1
myValues[52]=1
myModel.fit(myData, myValues)
Theta0 = myModel.intercept_
Theta1 = myModel.coef_
guess = myModel.decision_function(np.array([[10,10**2]]))
crossover = (-Theta1[0][0]-np.sqrt(Theta1[0][0]**2-4*Theta0*Theta1[0][1]))/(2*Theta1[0][1])
crossover2 = (-Theta1[0][0]+np.sqrt(Theta1[0][0]**2-4*Theta0*Theta1[0][1]))/(2*Theta1[0][1])