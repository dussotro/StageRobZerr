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

#creation de tout les proxy nécessaire
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

configEta = [["Frequency", 1.0],
          #BOTH FEET
          ["MaxStepX", 0.08],
          ["MaxStepFrequency", 0.4],
          
          # LEFT FOOT
          ["LeftStepHeight", 0.0022],
          ["LeftTorsoWx", -3*almath.TO_RAD],
          ["LeftTorsoWy", 3.0*almath.TO_RAD],
          
          # RIGHT FOOT
          ["RightStepHeight", 0.0022],
          ["RightTorsoWx", 3*almath.TO_RAD],
          ["RightTorsoWy", 3.0*almath.TO_RAD]]



def moveToEta(X, Y, Theta, Frequency):
    """
    In :
        X -> Distance à parcourir en X (face)
        Y -> Distance à parcourir en Y (cote)
        Theta -> Angle à parcourir    
        Frequency -> Vitesse du robot (entre 0 et 1)
    """ 
    
    #définition des booléens d'objectifs
    flag, flagX, flagY, flagTh = False, False, False, False
    
    #récupération de la position initiale
    initRobotPosition = almath.Pose2D(motionProxy.getRobotPosition(False))
    initRobotPosition = list(almath.transformFromPose2D(initRobotPosition).toVector())
    print(initRobotPosition)

    xi, yi, thi = initRobotPosition[3],initRobotPosition[7], np.arctan(initRobotPosition[1]/initRobotPosition[0])
    
    #calcul de la vitesse "optimale" en fonction de la distance à parcourir avec des minima en cas de courtes distances
    if X > 0:
        vX = np.log(1+X)/np.log(11)
        vX = max(0.5, vX)
    else :
        vX = - np.log(1-X)/(2*np.log(11))
        vX = min(-0.5, vX)
    
    if Y > 0:
        vY = np.log(Y + 1)/(5*np.log(3))
        vY = max(0.05, vY) 
    else:
        vY = -np.log(-Y + 1)/(5*np.log(3))
        vY = min(-0.05, vY) 
    
    if Theta > 0:
        omega = np.log(abs(Theta) + 1)/np.log(4.14)
    else :
        omega = - np.log(abs(Theta) + 1)/np.log(4.14)
        omega = min(-0.05, omega)
    #application des vitesses
    try:
        motionProxy.moveToward(vX, vY, omega, configEta)  
        pass
    except Exception, errorMsg:
        print str(errorMsg)
        print "This example is not allowed on this robot."
        exit()
    
    while not flag:
        print 'no flag raised'
        #mise a jour de la position spatiale du robot
        endRobotPosition = almath.Pose2D(motionProxy.getRobotPosition(False))
        endRobotPosition = list(almath.transformFromPose2D(endRobotPosition).toVector())
    
        xf, yf, thf = endRobotPosition[3], endRobotPosition[7], np.arctan(endRobotPosition[1]/endRobotPosition[0])
        print abs(xf - xi)
        
        #atteinte des objectifs
        if abs(xf - xi) > abs(X) :
            print 'FLAG X'
            vX = 0
            flagX = True
        if abs(yf - yi) > abs(Y) :
            print 'FLAG Y'
            vY = 0.2
            flagY = True
        if abs(thf - thi) > abs(Theta) :
            print 'FLAG THETA'
            omega = 0
            flagTh = True
        #atteinte de tout les ojectifs
        if flagX and flagY and flagTh:
            flag = True            
        motionProxy.moveToward(vX, vY, omega, configEta)  
        
    #remise à zero de la vitesse après atteinte des objectifs    
    motionProxy.moveToward(0.0, 0.0, 0.0)
    
    
def moveTowardEta(U, V, Omega, Frequency):
    
    """
    In :
        U -> Vitesse en X (face)
        Y -> Vitesse en Y (cote)
        Theta -> Angle à parcourir    
    
    """   

    
    motionProxy.moveToward(U, V, Omega, configEta)
    
    
if __name__ == '__main__':
    eta.doInitialisation()
    
#    try :
#        print ">>>>>>>>>>>>>> Test moveToEta"
#        moveToEta(0.57, 0, 0, 1.0)
#        postureProxy.goToPosture('Stand', 0.5)
#        time.sleep(3)
#               
#        print '>>>>>>>>>>>>>> Deuxieme étape'  
#        
#        moveTowardEta(-0.7, 0, 0, 1.0)
#        time.sleep(10)
#        
#        print '>>>>>>>>>>>>>> Fin'
#        
#    except Exception, e:
#        print "Value error: ", e

    enter = 0
#    while enter != -10000:
#        enter = int(input('RHipPitch: '))
#        motionProxy.setAngles("RHipRoll", enter ,0.3)
#        time.sleep(1)
    
    eta.doStop()
    
    
    
    