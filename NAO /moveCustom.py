#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 22 13:41:39 2017

@author: dussotro
"""

import time
import sys
from naoqi import ALProxy, ALModule
import motion
import select
import vision_showimages as vis
import numpy as np
import almath

import DiagNAO (eta).py as eta



try:
    motionProxy = ALProxy("ALMotion", robotIP, port)
except Exception, e:
    print"Could not create proxy to ALMotion"
    print"Error was: ", e
try:
    postureProxy = ALProxy("ALRobotPosture", robotIP, port)
except Exception, e:
    print "Could not create proxy to ALRobotPosture"
    print "Error was: ", e
try:
    sonarProxy = ALProxy("ALSonar", robotIP, port)
    sonarProxy.subscribe("myApplication")
except Exception, e:
    print "Could not create proxy to ALSonar"
    print "Error was: ", e

try :
    audio = ALProxy("ALAudioDevice", robotIP,port)
    audio.setOutputVolume(50)
except Exception, e: 
    print "Could not create proxy to ALaudioProxy"
    print "Error was: ", e
try :
    tts = ALProxy("ALTextToSpeech", robotIP, port)
    tts.setLanguage("French")
except Exception, e: 
    print "Could not create proxy to ALTextToSpeech"
    print "Error was: ", e

try:
    memoryProxy = ALProxy("ALMemory",robotIP, port)
except Exception, e:
    print "Could not create proxy to ALMemory"
    print "Error was: ", e
    
try:
    BatteryProxy = ALProxy("ALBattery",robotIP, port)
except Exception, e:
    print "Could not create proxy to AlBattery"
    print "Error was: ", e





def moveToEta(X, Y, Theta, Frequency):
    configEta = [["Frequency", Frequency],
          #BOTH FEET
          ["MaxStepX", 0.08],
          ["MaxStepFrequency", 0.5],
          
          # LEFT FOOT
          ["LeftStepHeight", 0.0022],
          ["LeftTorsoWx", -1*almath.TO_RAD],
          ["LeftTorsoWy", 3.0*almath.TO_RAD],
          
          # RIGHT FOOT
          ["RightStepHeight", 0.002],
          ["RightTorsoWx", 1*almath.TO_RAD],
          ["RightTorsoWy", 3.0*almath.TO_RAD]]
    flag = False
    
    initRobotPosition = almath.Pose2D(motionProxy.getRobotPosition(False))
    initRobotPosition = almath.transformFromPose2D(initRobotPosition)
    
    xi, yi, thi = initRobotPosition[0][3],initRobotPosition[1][3], np.arctan(initRobotPosition[0][1]/initRobotPosition[0][0])
    
    
    try:
        motionProxy.moveToward(vX, vY, omega, configEta)   

    except Exception, errorMsg:
        print str(errorMsg)
        print "This example is not allowed on this robot."
        exit()
    
    while not flag:
        
        endRobotPosition = almath.Pose2D(motionProxy.getRobotPosition(False))
        endRobotPosition = almath.transformFromPose2D(endRobotPosition)
    
        xf, yf, thf = endRobotPosition[0][3], endRobotPosition[1][3], np.arctan(endRobotPosition[0][1]/endRobotPosition[0][0])

        if abs(xf - xi) > abs(X) and abs(yf - yi) > abs(Y) and abs(thf - thi) > abs(Theta):
            flag = True            
    
    motionProxy.moveToward(0.0, 0.0, 0.0)
    
    
def moveTowardEta(U, V, Omega, Frequency):
    configEta = [["Frequency", Frequency],
          #BOTH FEET
          ["MaxStepX", 0.08],
          ["MaxStepFrequency", 0.5],
          
          # LEFT FOOT
          ["LeftStepHeight", 0.0022],
          ["LeftTorsoWx", -1*almath.TO_RAD],
          ["LeftTorsoWy", 3.0*almath.TO_RAD],
          
          # RIGHT FOOT
          ["RightStepHeight", 0.002],
          ["RightTorsoWx", 1*almath.TO_RAD],
          ["RightTorsoWy", 3.0*almath.TO_RAD]]
    
    motionProxy.moveToward(U, V, Omega, configEta)
    
    
if __name__ == '__main__':
    eta.doInitialisation()

    print "Test moveToEta"
    moveToEta(1, 0, 0, 0.1)
    
    time.sleep(2)
    moveTowardEta(0.5, 0, 0, 0.1)
    
    t0 = time.time()
    while time.time() - t0 < 5:
        continue
    
    moveTowardEta(0.0, 0.0, 0.0, 0.0)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    