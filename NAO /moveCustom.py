# -*- coding: utf-8 -*-
import time
import sys
from naoqi import ALProxy, ALModule
import motion
import select
import vision_showimages as vis
import numpy as np
import almath
import DiagNAO_eta as eta

robotIP = "172.20.28.103" #éta

port = 9559
CameraID = 0
Frequency = 0.0 #low speed
t=1.0

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
    tts.setLanguage("English")
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
    """
    In :
        X -> Distance à parcourir en X (face)
        Y -> Distance à parcourir en Y (cote)
        Theta -> Angle à parcourir    
        Frequency -> Vitesse du robot (entre 0 et 1)
    """    
    
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
    flag, flagX, flagY, flagTh = False, False, False, False
    
    initRobotPosition = almath.Pose2D(motionProxy.getRobotPosition(False))
    initRobotPosition = list(almath.transformFromPose2D(initRobotPosition).toVector())
    print(initRobotPosition)
    
    xi, yi, thi = initRobotPosition[3],initRobotPosition[7], np.arctan(initRobotPosition[1]/initRobotPosition[0])

        
    if X > 0:
        vX = np.log(1+X)/np.log(11)
    else :
        vX = - np.log(1-X)/(2*np.log(11))
    vX = max(0.4, vX)
    if Y > 0:
        vY = np.log(Y + 1)/(5*np.log(3))
    else:
        vY = -np.log(-Y + 1)/(5*np.log(3))
    vY = max(0.05, vY)
    if Theta > 0:
        omega = np.log(abs(Theta) + 1)/np.log(4.14)
    else :
        omega = - np.log(abs(Theta) + 1)/np.log(4.14)
    omega = max(0.05, omega)
    try:
        motionProxy.moveToward(vX, vY, omega, configEta)   

    except Exception, errorMsg:
        print str(errorMsg)
        print "This example is not allowed on this robot."
        exit()
    
    while not flag:          
        endRobotPosition = almath.Pose2D(motionProxy.getRobotPosition(False))
        endRobotPosition = list(almath.transformFromPose2D(endRobotPosition).toVector())
    
        xf, yf, thf = endRobotPosition[3], endRobotPosition[7], np.arctan(endRobotPosition[1]/endRobotPosition[0])
        print abs(xf - xi)
        
        if abs(xf - xi) > abs(X) :
            vX = 0
            flagX = True
        if abs(yf - yi) > abs(Y) :
            vY = 0
            flagY = True
        if abs(thf - thi) > abs(Theta) :
            omega = 0
            flagTh = True
            
        if flagX and flagY and flagTh:
            flag = True            
        
        motionProxy.moveToward(vX, vY, omega, configEta)
        
    motionProxy.moveToward(0.0, 0.0, 0.0)
    
    
def moveTowardEta(U, V, Omega, Frequency):
    """
    In :
        U -> Vitesse en X (face)
        Y -> Vitesse en Y (cote)
        Theta -> Angle à parcourir    
    
    """   
    
    
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
    
    try :
        print "Test moveToEta"
        #moveToEta(2, 0, 0, 1)
        pass
    except Exception, e:
        print "Value error: ", e
    
    eta.doStop()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    