# -*- encoding: UTF-8 -*-
import time
import sys
from naoqi import ALProxy, ALModule, ALBroker
import motion
import select
#import vision_showimages as vis
import numpy as np
import almath
from PyQt4.QtGui import QWidget, QImage, QApplication, QPainter, QPushButton
import signal
from optparse import OptionParser

robotIP = "172.20.12.210" #Nabotbleudeu
#robotIP = "172.20.13.107" #Eta
#robotIP = "172.20.13.63" #Rouge
#robotIP = "172.20.28.103" #Bleu
#robotIP = "172.20.11.237"# gamma 
#robotIP = "172.20.11.242"# beta

port = 9559
CameraID = 0
Frequency = 0.0 #low speed
t=1.0

"""
Fichier ayant pour but de lancer un suivie de son par un NAO Bleue.
Le Nao dera se rendre vers le son émit. 
les résultat sont approximatif. Néanmoins si le son est suffisament net,
fort alors ce sera moins approximatif et plus cohérent.
"""

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
    audio.setOutputVolume(90)
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

try : 
    AutonomousMoves = ALProxy("ALAutonomousMoves",robotIP,port)
except Exception, e :
    print"Could not create proxy to ALAutonomousMoves"
    print "Error was : ", e
AutonomousMoves.setExpressiveListeningEnabled(False)

try :
    SoundLocalization = ALProxy("ALSoundLocalization", robotIP,port)
except Exception, e :
    print "Error was : ",e

try : 
    tracker = ALProxy("ALTracker", robotIP, port)
except Exception, e:
    print "Error was :",e



#stiffness for real NAO Robot
def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 0.6
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)
    
def doInitialisation():
    print">>>>>> Initialisation"
#     Set NAO in Stiffness On
    StiffnessOn(motionProxy)
#     Send NAO to Pose Init
    postureProxy.goToPosture("StandInit", 0.2)
    
def doStandUp():
    
    motionProxy.wakeUp()
    motionProxy.setStiffnesses("Body", 1.0)
    time.sleep(t)
    print"standing up"


def initmouv():
    postureProxy.goToPosture("Crouch", 0.2)
#    names = ['HipYawPitch','RKneePitch','LKneePitch' ,'LHipPitch','RHipPitch','RAnklePitch','LAnklePitch']
#    kneeAngle = 2.1
#    angles = [0.0, kneeAngle, kneeAngle , -kneeAngle/2 , -kneeAngle/2 ,-kneeAngle/2, -kneeAngle/2 ]
    listValStandInit = [memoryProxy.getData("Device/SubDeviceList/HeadYaw/Position/Actuator/Value"),
                    memoryProxy.getData("Device/SubDeviceList/HeadPitch/Position/Actuator/Value"),
                    memoryProxy.getData("Device/SubDeviceList/LShoulderPitch/Position/Actuator/Value"),
                    memoryProxy.getData("Device/SubDeviceList/LShoulderRoll/Position/Actuator/Value"),
                    memoryProxy.getData("Device/SubDeviceList/LElbowYaw/Position/Actuator/Value"),
                    memoryProxy.getData("Device/SubDeviceList/LElbowRoll/Position/Actuator/Value"),
                    memoryProxy.getData("Device/SubDeviceList/LWristYaw/Position/Actuator/Value"),
                    memoryProxy.getData("Device/SubDeviceList/LHand/Position/Actuator/Value"),
                    memoryProxy.getData("Device/SubDeviceList/LKneePitch/Position/Actuator/Value"),
                    0,
                    0]

    tab = [0,0,0,0,0,0,0,0,120,0,0]
    
    listValStandInit = sumList(listValStandInit, tab)     
    configRob(listValStandInit[0], listValStandInit[1], listValStandInit[2], listValStandInit[3], listValStandInit[4], listValStandInit[5], listValStandInit[6], listValStandInit[7], listValStandInit[8], listValStandInit[9], listValStandInit[10])
    
    time.sleep(0.5)

    

#==============================================================================
# test de toutes les articulations
#==============================================================================



def configRob(HeadYawAngle, HeadPitchAngle, ShoulderPitchAngle, ShoulderRollAngle, ElbowYawAngle, ElbowRollAngle,WristYawAngle, HandAngle, kneeAngle, torsoAngle, spreadAngle):
    robotConfig = motionProxy.getRobotConfig()
    robotName = ""
    for i in range(len(robotConfig[0])):
        if (robotConfig[0][i] == "Model Type"):
            robotName = robotConfig[1][i]
    
        if robotName == "naoH25":
    
            Head     = [HeadYawAngle, HeadPitchAngle]
    
            LeftArm  = [ShoulderPitchAngle, +ShoulderRollAngle, +ElbowYawAngle, +ElbowRollAngle, +WristYawAngle, HandAngle]
            RightArm = [ShoulderPitchAngle, -ShoulderRollAngle, -ElbowYawAngle, -ElbowRollAngle, -WristYawAngle, HandAngle]
    
            LeftLeg  = [0.0,                      #hipYawPitch
                        spreadAngle,              #hipRoll
                        -kneeAngle/2-torsoAngle,  #hipPitch
                        kneeAngle,                #kneePitch
                        -kneeAngle/2,             #anklePitch
                        -spreadAngle]             #ankleRoll
            RightLeg = [0.0, -spreadAngle, -kneeAngle/2-torsoAngle, kneeAngle, -kneeAngle/2,  spreadAngle]
    
        elif robotName == "naoH21":
    
            Head     = [HeadYawAngle, HeadPitchAngle]
    
            LeftArm  = [ShoulderPitchAngle, +ShoulderRollAngle, +ElbowYawAngle, +ElbowRollAngle]
            RightArm = [ShoulderPitchAngle, -ShoulderRollAngle, -ElbowYawAngle, -ElbowRollAngle]
    
            LeftLeg  = [0.0,  spreadAngle, -kneeAngle/2-torsoAngle, kneeAngle, -kneeAngle/2, -spreadAngle]
            RightLeg = [0.0, -spreadAngle, -kneeAngle/2-torsoAngle, kneeAngle, -kneeAngle/2,  spreadAngle]
    
        elif robotName == "naoT14":
    
            Head     = [HeadYawAngle, HeadPitchAngle]
    
            LeftLeg  = [0.0,  spreadAngle, -kneeAngle/2-torsoAngle, kneeAngle, -kneeAngle/2, -spreadAngle]
            RightLeg = [0.0, -spreadAngle, -kneeAngle/2-torsoAngle, kneeAngle, -kneeAngle/2,  spreadAngle]
    
            LeftLeg  = []
            RightLeg = []
    
        elif robotName == "naoT2":
    
            Head     = [HeadYawAngle, HeadPitchAngle]
    
            LeftArm  = []
            RightArm = []
    
            LeftLeg  = []
            RightLeg = []
    
        else:
            print "ERROR : Your robot is unknown"
            print "This test is not available for your Robot"
            print "---------------------"
            exit(1)
    
        # Gather the joints together
        pTargetAngles = Head + LeftArm + LeftLeg + RightLeg + RightArm
    
        # Convert to radians
        pTargetAngles = [ x * almath.TO_RAD for x in pTargetAngles]

        # We use the "Body" name to signify the collection of all joints
        pNames = "Body"
        # We set the fraction of max speed
        pMaxSpeedFraction = 0.35
        # Ask motion to do this with a blocking call    
        motionProxy.angleInterpolationWithSpeed(pNames, pTargetAngles, pMaxSpeedFraction)

def Test_Articulations(queue=None,prog=None):
    StiffnessOn(motionProxy)


    # Send NAO to Pose Init
    postureProxy.goToPosture("StandZero", 2.0)
    # Get the Robot Configuration    
    listValStandInit = [memoryProxy.getData("Device/SubDeviceList/HeadYaw/Position/Actuator/Value"),
                        memoryProxy.getData("Device/SubDeviceList/HeadPitch/Position/Actuator/Value"),
                        memoryProxy.getData("Device/SubDeviceList/LShoulderPitch/Position/Actuator/Value"),
                        memoryProxy.getData("Device/SubDeviceList/LShoulderRoll/Position/Actuator/Value"),
                        memoryProxy.getData("Device/SubDeviceList/LElbowYaw/Position/Actuator/Value"),
                        memoryProxy.getData("Device/SubDeviceList/LElbowRoll/Position/Actuator/Value"),
                        memoryProxy.getData("Device/SubDeviceList/LWristYaw/Position/Actuator/Value"),
                        memoryProxy.getData("Device/SubDeviceList/LHand/Position/Actuator/Value"),
                        memoryProxy.getData("Device/SubDeviceList/LKneePitch/Position/Actuator/Value"),
                        0,
                        0]
    
    tab = [[[0,0,0,0,0,0,0,0,120,0,0],[0,0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0,0]],
           [[130,0,0,0,0,0,0,0,0,0,0],[-260,0,0,0,0,0,0,0,0,0,0], [130,0,0,0,0,0,0,0,0,0,0]],
           [[0,30,0,0,0,0,0,0,0,0,0],[0,-70,0,0,0,0,0,0,0,0,0], [0,40,0,0,0,0,0,0,0,0,0]],
           [[0,0,120,0,0,0,0,0,0,0,0],[0,0,-240,0,0,0,0,0,0,0,0], [0,0,120,0,0,0,0,0,0,0,0]],
           [[0,0,0,-18,0,0,0,0,0,0,0],[0,0,0,95,0,0,0,0,0,0,0], [0,0,0,-75,0,0,0,0,0,0,0]],
           [[0,0,0,0,120,0,0,0,0,0,0],[0,0,0,0,-240,0,0,0,0,0,0], [0,0,0,0,120,0,0,0,0,0,0]],
           [[0,0,0,0,0,0,-2,0,0,0,0,0],[0,0,0,0,0,90,0,0,0,0,0], [0,0,0,0,0,0,-88,0,0,0,0]],
           [[0,0,0,0,0,0,105,0,0,0,0],[0,0,0,0,0,0,-210,0,0,0,0], [0,0,0,0,0,0,105,0,0,0,0]]]
    
    for i in range(len(tab)):
        for j in range(3):
            listValStandInit = sumList(listValStandInit, tab[i][j])
            configRob(listValStandInit[0], listValStandInit[1], listValStandInit[2], listValStandInit[3], listValStandInit[4], listValStandInit[5], listValStandInit[6], listValStandInit[7], listValStandInit[8], listValStandInit[9], listValStandInit[10])
    time.sleep(2)
    motionProxy.openHand('RHand')
    motionProxy.openHand('LHand')
    time.sleep(1)
    motionProxy.closeHand('RHand')
    motionProxy.closeHand('LHand')
    initmouv()
    prog.value = 0
    
def sumList(a, b):
    result = []
    for i in range(len(a)):
        result.append(a[i] + b[i])
    return result


#==============================================================================
# Audio
#==============================================================================
def TestTts(texte):
    tts.say(texte)
    
#==============================================================================
# """Vision"""
#==============================================================================

#def Test_Image():
#
#    ####
#    # Create proxy on ALVideoDevice
#
#    print "Creating ALVideoDevice proxy to ", robotIP
#
#    camProxy = ALProxy("ALVideoDevice", robotIP, port)
#
#    ####
#    # Register a Generic Video Module
#
#    resolution = vision_definitions.kQVGA
#    colorSpace = vision_definitions.kYUVColorSpace
#    fps = 30
#
#    nameId = camProxy.subscribe("python_GVM", resolution, colorSpace, fps)
#    print nameId
#
#    print 'getting images in local'
#    for i in range(0, 20):
#      camProxy.getImageLocal(nameId)
#      camProxy.releaseImage(nameId)
#
#    resolution = vision_definitions.kQQVGA
#    camProxy.setResolution(nameId, resolution)
#
#    print 'getting images in remote'
#    for i in range(0, 20):
#      camProxy.getImageRemote(nameId)
#
#    camProxy.unsubscribe(nameId)
#
#    print 'end of gvm_getImageLocal python script'
#
#def showNaoImage():
#    videoRecorderProxy = ALProxy("ALVideoRecorder", robotIP, port)
#    
#    # This records a 320*240 MJPG video at 10 fps.
#    # Note MJPG can't be recorded with a framerate lower than 3 fps.
#    videoRecorderProxy.setResolution(1) 
#    videoRecorderProxy.setFrameRate(10)
#    videoRecorderProxy.setVideoFormat("MJPG")
#    videoRecorderProxy.startRecording("./", "test")
#
#    time.sleep(5)
#    # Video file is saved on the robot in the
#    # /home/nao/recordings/cameras/ folder.
#    videoInfo = videoRecorderProxy.stopRecording()
#    #print type video
#    print "Video was saved on the robot: ", videoInfo[1]
#    print "Num frames: ", videoInfo[0]
#    video = memoryProxy.getData("./test.avi")


#==============================================================================
# """Sensors"""
#==============================================================================
def TrySensors():
    SLeft = [memoryProxy.getData("Device/SubDeviceList/US/Left/Sensor/Value")]
    SRight = [memoryProxy.getData("Device/SubDeviceList/US/Right/Sensor/Value")]
    for i in range (5):
        SLeft.append(memoryProxy.getData("Device/SubDeviceList/US/Left/Sensor/Value"))
        SRight.append(memoryProxy.getData("Device/SubDeviceList/US/Right/Sensor/Value"))
    Left = np.mean(SLeft)
    Right = np.mean(SRight)
#    print 'Left :', Left
#    print 'Right:', Right
    return Left, Right

def Accelero():
    X = memoryProxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value")
    Y = memoryProxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value") 
    Z = memoryProxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value")
#    print "X = ", X
#    print "Y = ", Y
#    print "Z = ", Z
    
    AngleX = memoryProxy.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value")
    AngleY = memoryProxy.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value")
    return AngleX, AngleY

    
#==============================================================================
# """Motion"""
#==============================================================================
config_robo_back = [["Frequency", 0.4],
               ["MaxStepX", 0.04],
               ["RightMaxStepFrequency", 0.3],
               ["RightStepHeight", 0.0018],
               ["RightTorsoWx",3*almath.TO_RAD],
               ["RightTorsoWy", 1.0*almath.TO_RAD],
               ["LeftMaxStepFrequency", 0.1],
               ["LeftStepHeight", 0.0018],
               ["LeftTorsoWx", -3*almath.TO_RAD],
               ["LeftTorsoWy", 1.0*almath.TO_RAD]  ]
config_robo = [["Frequency", 0.4],
               ["MaxStepX", 0.04],
               ["RightMaxStepFrequency", 0.3],
               ["RightStepHeight", 0.0018],
               ["RightTorsoWx",3*almath.TO_RAD],
               ["RightTorsoWy", 1.0*almath.TO_RAD],
               ["LeftMaxStepFrequency", 0.1],
               ["LeftStepHeight", 0.0018],
               ["LeftTorsoWx", -3*almath.TO_RAD],
               ["LeftTorsoWy", 1.0*almath.TO_RAD]  ]

def dorun(t):
    X = 0.9
    Y = 0.0
    Theta = 0.0
#    Frequency =0.9 # low speed
    motionProxy.moveTo(X, Y, Theta)
#    
    t0 = time.time()
    AngX, AngY = [], []
    while time.time()< (t0 + t):
        accelero = Accelero()
        AngX.append(accelero[0])
        AngY.append(accelero[1])
        time.sleep(0.2)
#    motionProxy.moveTo (0.4, 0, 0)
#    time.sleep(t)
    maxAngX, maxAngY = max(AngX), max(AngY)
    print "maxAngX, maxAngY = ", maxAngX, ", ", maxAngY
    motionProxy.setWalkTargetVelocity(0.0, 0, 0, 1)
    print"running"
    

def doback():
    
    X = -0.4
    Y = 0.0
    Theta = 0.0
#    Frequency =0.9 # low speed
    motionProxy.moveTo(X, Y, Theta)
     
    time.sleep(t)
    print"back"
    
def doleft(angle):
    
    initRobotPosition = almath.Pose2D(motionProxy.getRobotPosition(False))
    
    theta= angle
#    motionProxy.setWalkTargetVelocity(0, 0, 0.5, 0.01)
    motionProxy.moveTo (0, 0, theta)
    #####################
    ## get robot position after move
    #####################
    endRobotPosition = almath.Pose2D(motionProxy.getRobotPosition(False))

    #####################
    ## compute and print the robot motion
    #####################
    robotMove = almath.pose2DInverse(initRobotPosition)*endRobotPosition
    print "virage gauche :", robotMove




    time.sleep(t)
    print"turning left"

def doright(angle):
    
    initRobotPosition = almath.Pose2D(motionProxy.getRobotPosition(False))
    
    theta= -angle
    motionProxy.moveTo (0, 0, theta)
#    motionProxy.setWalkTargetVelocity(0, 0, -0.5, 0.01)
    #####################
    ## get robot position after move
    #####################
    endRobotPosition = almath.Pose2D(motionProxy.getRobotPosition(False))

    #####################
    ## compute and print the robot motion
    #####################
    robotMove = almath.pose2DInverse(initRobotPosition)*endRobotPosition
    print "virage droite :", robotMove


    time.sleep(t)
    print"turning right"
    

    
def doStop():
    time.sleep(0.2)
    motionProxy.setWalkTargetVelocity(0, 0, 0, Frequency)
    postureProxy.goToPosture("Crouch", 0.2)
    Accelero()
    motionProxy.setStiffnesses("Body", 0.0)
    motionProxy.rest()
    time.sleep(0.5)
    print"Stopping"


def target_velocity():
    #TARGET VELOCITY
    X = 0.4
    Y = 0.0
    Theta = 0.0
    Frequency =0.4 # max speed
    motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)

    time.sleep(4.0)
    print "Straight Forward"
    print "walk Speed X :",motionProxy.getRobotVelocity()[0]," m/s"
    
    X = -0.4  #backward
    Y = 0.0
    Theta = 0.0
    Frequency =0.4 # low speed
    motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)
    
    time.sleep(4.0)
    print "Straight Backward"
    print "walk Speed X :",motionProxy.getRobotVelocity()[0]," m/s"

def position_robot():
    
    initRobotPosition = almath.Pose2D(motionProxy.getRobotPosition(False))

    X = 0.3
    Y = 0.1
    Theta = np.pi/2.0
    motionProxy.post.moveTo(X, Y, Theta)
    # wait is useful because with post moveprint " left_foot:",left_foot
#    To is not blocking function
    motionProxy.waitUntilMoveIsFinished()

    #####################
    ## get robot position after move
    #####################
    endRobotPosition = almath.Pose2D(motionProxy.getRobotPosition(False))

    #####################
    ## compute and print the robot motion
    #####################
    robotMove = almath.pose2DInverse(initRobotPosition)*endRobotPosition
    print "Robot Move :", robotMove
    

    
def Test_Square_Left_Right():
    print ">>>>>>>>>>> Test du carre"
    print ">>> carre gauche"
    time.sleep(2)
    for i in range(4):
        dorun(6)
        BatteryMemory()
        doleft(np.pi/2)
    time.sleep(2)
    print ">>> carre droite"
    for j in range(4):
        dorun(6)
        BatteryMemory()
        doright(np.pi/2)
    print "fin de test du carre"
    BatteryMemory()

#==============================================================================
# Battery
#==============================================================================
def BatteryMemory():
    percentage = memoryProxy.getData("Device/SubDeviceList/Battery/Current/Sensor/Value") 
    c = memoryProxy.getData ("Device/SubDeviceList/Battery/Charge/Sensor/Value")
    b = memoryProxy.getData ("Device/SubDeviceList/Battery/Charge/Sensor/Status")
    s = []
    t0 = time.time()
    while time.time() - t0 < 1:
        s.append(c)
        c = memoryProxy.getData ("Device/SubDeviceList/Battery/Charge/Sensor/Value")
    #print "Percentage = ", percentage    
    print "Pourcentage de la batterie :", np.mean(s) * 100, "%"
    #print "status =", b
    return np.mean(s) * 100
     

#==============================================================================
# definitions des fonctions articulations  de robot (elles sont utiles pour l interface graphique)
#============================================================================== 
    
def Tete(queue=None,prog=None):
    initmouv()
    names = ['HeadYaw']
    angles = [-2.0857]
    motionProxy.setAngles(names,angles,.6)
    time.sleep(1.5)
    names = ['HeadYaw']
    angles = [+2.0857]
    motionProxy.setAngles(names,angles,.6)
    time.sleep(1.5)
    names = ['HeadYaw']
    angles = [0]
    motionProxy.setAngles(names,angles,.6)
    time.sleep(1.5)
    names = ['HeadPitch']
    angles = [-0.671951]
    motionProxy.setAngles(names,angles,.6)
    time.sleep(1)
    names = ['HeadPitch']
    angles = [0.515047]
    motionProxy.setAngles(names,angles,.6)
    time.sleep(1)
    names = ['HeadPitch']
    angles = [0]
    motionProxy.setAngles(names,angles,.6)
    time.sleep(1)
    initmouv()
    prog.value = 0
    

def Epaule(queue=None,prog=None):
    initmouv()
    names = ['LShoulderPitch']
    angles = [2.0857]
    motionProxy.setAngles(names,angles,.35)
#    time.sleep(2)
    names = ['RShoulderPitch']
    angles = [2.0857]
    motionProxy.setAngles(names,angles,.35)
    time.sleep(2)
    names = ['LShoulderPitch']
    angles = [-2.0857]
    motionProxy.setAngles(names,angles,.35)
#    time.sleep(2)
    names = ['RShoulderPitch']
    angles = [-2.0857]
    motionProxy.setAngles(names,angles,.35)
    time.sleep(2)
    names = ['LShoulderPitch']
    angles = [0]
    motionProxy.setAngles(names,angles,.35)
#    time.sleep(2)  
    names = ['RShoulderPitch']
    angles = [0]
    motionProxy.setAngles(names,angles,.35)
    time.sleep(2)
    
    names = ['LShoulderRoll','RShoulderRoll']
    angles = [-0.3142,0.3142]
    motionProxy.setAngles(names,angles,.35)
    time.sleep(0.5)
    names = ['LShoulderRoll','RShoulderRoll']
    angles =  [1.3265, 	-1.3265 ]
    time.sleep(1)
    motionProxy.setAngles(names,angles,.35)
    names = ['LShoulderRoll','RShoulderRoll']
    angles =  [0, 0 ]
    time.sleep(1)
    motionProxy.setAngles(names,angles,.35)
    prog.value = 0
    
def Coudes(queue=None,prog=None):
    initmouv()
    names = ['LElbowRoll']
    angles = [-1.5446 ]
    motionProxy.setAngles(names,angles,.5)
#    time.sleep(2)
    names = ['RElbowRoll']
    angles = [+1.5446 ]
    motionProxy.setAngles(names,angles,.5)
    time.sleep(1)
    names = ['LElbowRoll']
    angles = [-0.0349]
    motionProxy.setAngles(names,angles,.5)
#    time.sleep(2)
    names = ['RElbowRoll']
    angles = [+0.0349]
    motionProxy.setAngles(names,angles,.5)
    time.sleep(1)
    names = ['LElbowRoll']
    angles = [0]
    motionProxy.setAngles(names,angles,.5)
#    time.sleep(2)  
    names = ['RElbowRoll']
    angles = [0]
    motionProxy.setAngles(names,angles,.5)
    time.sleep(2)
#    prog.value = 0
    
    time.sleep(1)    
    names = ['LElbowYaw','RElbowYaw']
    angles = [-2.0857 ,2.0857]
    motionProxy.setAngles(names,angles,.5)
    time.sleep(2)
    names = ['LElbowYaw','RElbowYaw']
    angles = [2.0857 ,-2.0857]
    motionProxy.setAngles(names,angles,.5)
    time.sleep(1)
    names = ['LElbowYaw','RElbowYaw']
    angles = [0 ,0]
    motionProxy.setAngles(names,angles,.5)
    time.sleep(1)
    prog.value = 0    
    
def Poignet(queue=None,prog=None):
    initmouv()
    names  = ['LWristYaw','RWristYaw']
    angles = [ -1.8238, +1.8238]
    motionProxy.setAngles(names,angles,.2)
    time.sleep(2)
    names  = ['LWristYaw','RWristYaw']
    angles = [ +1.8238, -1.8238]
    motionProxy.setAngles(names,angles,.2)
    time.sleep(2)
    names  = ['LWristYaw','RWristYaw']
    angles = [ 0, 0]
    motionProxy.setAngles(names,angles,.2)
    time.sleep(2)
    prog.value = 0
    
def Main(queue=None,prog=None):
    initmouv()
    motionProxy.openHand('RHand')
    motionProxy.openHand('LHand')
    time.sleep(1)
    motionProxy.closeHand('RHand')
    motionProxy.closeHand('LHand')
    prog.value = 0
    
#==============================================================================
# prend les valeurs de capteurs pied d un robot
#============================================================================== 

def fsr():
   left_foot= [ memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value"),
    memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value"),
    memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value"),
    memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value")]
    
   right_foot= [ memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value"),
    memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value"),
    memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value"),
    memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")]
   
   print " left_foot:",left_foot
   print " right_foot:",right_foot
   
 
def gyroscope():
    
    a = memoryProxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value")
    b = memoryProxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value")
    c = memoryProxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value")
    print "a", a
    print "b", b
    print "c", c

#==============================================================================
# fait certains mouvements par les mains et les pieds de robot
#============================================================================== 

# fait bouger la maine
def userArmArticular(motionProxy):
    # Arms motion from user have always the priority than walk arms motion
    JointNames = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll"]
    Arm1 = [-40,  25, 0, -40]
    Arm1 = [ x * motion.TO_RAD for x in Arm1]

    Arm2 = [-40,  50, 0, -80]
    Arm2 = [ x * motion.TO_RAD for x in Arm2]

    pFractionMaxSpeed = 0.6

    motionProxy.angleInterpolationWithSpeed(JointNames, Arm1, pFractionMaxSpeed)
    motionProxy.angleInterpolationWithSpeed(JointNames, Arm2, pFractionMaxSpeed)
    motionProxy.angleInterpolationWithSpeed(JointNames, Arm1, pFractionMaxSpeed)
    
def userArmArticular_r(motionProxy):
    # Arms motion from user have always the priority than walk arms motion
    JointNames = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"]
    Arm1 = [-40,  25, 0, -40]
    Arm1 = [ x * motion.TO_RAD for x in Arm1]

    Arm2 = [-40,  50, 0, 80]
    Arm2 = [ x * motion.TO_RAD for x in Arm2]

    pFractionMaxSpeed = 0.6

    motionProxy.angleInterpolationWithSpeed(JointNames, Arm1, pFractionMaxSpeed)
    motionProxy.angleInterpolationWithSpeed(JointNames, Arm2, pFractionMaxSpeed)
    motionProxy.angleInterpolationWithSpeed(JointNames, Arm1, pFractionMaxSpeed)
    
    
def steps():
    footStepsList = []

    # 1) Step forward with your lefIf time is zero, no alarm is scheduled, and any scheduled alarm is canceled. If the return value is zero, no alarm is currently scheduled.t foot
    footStepsList.append([["LLeg"], [[0.06, 0.1, 0.0]]])

    # 2) Sidestep to the left with your left foot
    footStepsList.append([["LLeg"], [[0.00, 0.16, 0.0]]])

    # 3) Move your right foot to your left foot
    footStepsList.append([["RLeg"], [[0.00, -0.1, 0.0]]])

    # 4) Sidestep to the left with your left foot
    footStepsList.append([["LLeg"], [[0.00, 0.16, 0.0]]])

    # 5) Step backward & left with your right foot
    footStepsList.append([["RLeg"], [[-0.04, -0.1, 0.0]]])

    # 6)Step forward & right with your right foot
    footStepsList.append([["RLeg"], [[0.00, -0.16, 0.0]]])

    # 7) Move your left foot to your right foot
    footStepsList.append([["LLeg"], [[0.00, 0.1, 0.0]]])

    # 8) Sidestep to the right with your right foot
    footStepsList.append([["RLeg"], [[0.00, -0.16, 0.0]]])

    ###############################
    # Send Foot step
    ###############################
    stepFrequency = 0.8
    clearExisting = False
    nbStepDance = 2 # defined the number of cycle to make

    for j in range( nbStepDance ):
        for i in range( len(footStepsList) ):
            motionProxy.setFootStepsWithSpeed(
                footStepsList[i][0],
                footStepsList[i][1],
                [stepFrequency],
                clearExisting)
            
#==============================================================================
# utilisation d autre fonction pour faire bouger le robot
#============================================================================== 
def towalk():
    x  = 1.0
    y  = 0.0
    theta  = 0.0
    moveConfig = [["Frequency", 1.0]]
    motionProxy.moveToward(x, y, theta, moveConfig)
    # If we don't send another command, he will walk forever
    # Lets make him slow down (step length) and turn after 10 seconds
    time.sleep(10)
    x = 0.5
    theta = 0.6
    motionProxy.moveToward(x, y, theta, moveConfig)
    # Lets make him slow down(frequency) after 5 seconds
    time.sleep(5)
    moveConfig = [["Frequency", 0.5]]
    motionProxy.moveToward(x, y, theta, moveConfig)
    # After another 10 seconds, we'll make him stop
    time.sleep(10)
    motionProxy.moveToward(0.0, 0.0, 0.0)

def dancer():
    
    
    # Set NAO in Stiffness On
#    StiffnessOn(proxy)

    # Send NAO to Pose Init
    postureProxy.goToPosture("StandInit", 0.5)

    # Enable Whole Body Balancer
    isEnabled  = True
    motionProxy.wbEnable(isEnabled)

    # Legs are constrained fixed
    stateName  = "Fixed"
    supportLeg = "Legs"
    motionProxy.wbFootState(stateName, supportLeg)

    # Constraint Balance Motion
    isEnable   = True
    supportLeg = "Legs"
    motionProxy.wbEnableBalanceConstraint(isEnable, supportLeg)

    # Arms motion
    effectorList = ["LArm", "RArm"]

    space        = motion.FRAME_ROBOT

    pathList     = [
                    [
                     [0.0,   0.08,  0.14, 0.0, 0.0, 0.0], # target 1 for "LArm"
                     [0.0,  -0.05, -0.07, 0.0, 0.0, 0.0], # target 2 for "LArm"
                     [0.0,   0.08,  0.14, 0.0, 0.0, 0.0], # target 3 for "LArm"
                     [0.0,  -0.05, -0.07, 0.0, 0.0, 0.0], # target 4 for "LArm"
                     [0.0,   0.08,  0.14, 0.0, 0.0, 0.0], # target 5 for "LArm"
                     ],
                    [
                     [0.0,   0.05, -0.07, 0.0, 0.0, 0.0], # target 1 for "RArm"
                     [0.0,  -0.08,  0.14, 0.0, 0.0, 0.0], # target 2 for "RArm"
                     [0.0,   0.05, -0.07, 0.0, 0.0, 0.0], # target 3 for "RArm"
                     [0.0,  -0.08,  0.14, 0.0, 0.0, 0.0], # target 4 for "RArm"
                     [0.0,   0.05, -0.07, 0.0, 0.0, 0.0], # target 5 for "RArm"
                     [0.0,  -0.08,  0.14, 0.0, 0.0, 0.0], # target 6 for "RArm"
                     ]
                    ]

    axisMaskList = [almath.AXIS_MASK_VEL, # for "LArm"
                    almath.AXIS_MASK_VEL] # for "RArm"

    coef       = 1.5
    timesList  = [ [coef*(i+1) for i in range(5)],  # for "LArm" in seconds
                   [coef*(i+1) for i in range(6)] ] # for "RArm" in seconds

    isAbsolute   = False

    # called cartesian interpolation
    motionProxy.positionInterpolations(effectorList, space, pathList,
                                 axisMaskList, timesList, isAbsolute)

    # Torso Motion
    effectorList = ["Torso", "LArm", "RArm"]

    dy = 0.06
    dz = 0.06
    pathList     = [
                    [
                     [0.0, +dy, -dz, 0.0, 0.0, 0.0], # target  1 for "Torso"
                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # target  2 for "Torso"
                     [0.0, -dy, -dz, 0.0, 0.0, 0.0], # target  3 for "Torso"
                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # target  4 for "Torso"
                     [0.0, +dy, -dz, 0.0, 0.0, 0.0], # target  5 for "Torso"
                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # target  6 for "Torso"
                     [0.0, -dy, -dz, 0.0, 0.0, 0.0], # target  7 for "Torso"
                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # target  8 for "Torso"
                     [0.0, +dy, -dz, 0.0, 0.0, 0.0], # target  9 for "Torso"
                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # target 10 for "Torso"
                     [0.0, -dy, -dz, 0.0, 0.0, 0.0], # target 11 for "Torso"
                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # target 12 for "Torso"
                     ],
                     [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], # for "LArm"
                     [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], # for "LArm"
                    ]

    axisMaskList = [almath.AXIS_MASK_ALL, # for "Torso"
                    almath.AXIS_MASK_VEL, # for "LArm"
                    almath.AXIS_MASK_VEL] # for "RArm"

    coef       = 0.5
    timesList  = [
                  [coef*(i+1) for i in range(12)], # for "Torso" in seconds
                  [coef*12],                       # for "LArm" in seconds
                  [coef*12]                        # for "RArm" in seconds
                 ]

    isAbsolute   = False

    motionProxy.positionInterpolations(effectorList, space, pathList, 
                                      axisMaskList, timesList, isAbsolute)


    # Deactivate whole body
    isEnabled    = False
    motionProxy.wbEnable(isEnabled)

    # Send NAO to Pose Init
    postureProxy.goToPosture("StandInit", 0.5)
 
def dancer():
    
    
    # Set NAO in Stiffness On
#    StiffnessOn(proxy)

    # Send NAO to Pose Init
    postureProxy.goToPosture("StandInit", 0.5)

    # Enable Whole Body Balancer
    isEnabled  = True
    motionProxy.wbEnable(isEnabled)

    # Legs are constrained fixed
    stateName  = "Fixed"
    supportLeg = "Legs"
    motionProxy.wbFootState(stateName, supportLeg)

    # Constraint Balance Motion
    isEnable   = True
    supportLeg = "Legs"
    motionProxy.wbEnableBalanceConstraint(isEnable, supportLeg)

    # Arms motion
    effectorList = ["LArm", "RArm"]

    space        = motion.FRAME_ROBOT

    pathList     = [
                    [
                     [0.0,   0.08,  0.14, 0.0, 0.0, 0.0], # target 1 for "LArm"
                     [0.0,  -0.05, -0.07, 0.0, 0.0, 0.0], # target 2 for "LArm"
                     [0.0,   0.08,  0.14, 0.0, 0.0, 0.0], # target 3 for "LArm"
                     [0.0,  -0.05, -0.07, 0.0, 0.0, 0.0], # target 4 for "LArm"
                     [0.0,   0.08,  0.14, 0.0, 0.0, 0.0], # target 5 for "LArm"
                     ],
                    [
                     [0.0,   0.05, -0.07, 0.0, 0.0, 0.0], # target 1 for "RArm"
                     [0.0,  -0.08,  0.14, 0.0, 0.0, 0.0], # target 2 for "RArm"
                     [0.0,   0.05, -0.07, 0.0, 0.0, 0.0], # target 3 for "RArm"
                     [0.0,  -0.08,  0.14, 0.0, 0.0, 0.0], # target 4 for "RArm"
                     [0.0,   0.05, -0.07, 0.0, 0.0, 0.0], # target 5 for "RArm"
                     [0.0,  -0.08,  0.14, 0.0, 0.0, 0.0], # target 6 for "RArm"
                     ]
                    ]

    axisMaskList = [almath.AXIS_MASK_VEL, # for "LArm"
                    almath.AXIS_MASK_VEL] # for "RArm"

    coef       = 1.5
    timesList  = [ [coef*(i+1) for i in range(5)],  # for "LArm" in seconds
                   [coef*(i+1) for i in range(6)] ] # for "RArm" in seconds

    isAbsolute   = False

    # called cartesian interpolation
    motionProxy.positionInterpolations(effectorList, space, pathList,
                                 axisMaskList, timesList, isAbsolute)

    # Torso Motion
    effectorList = ["Torso", "LArm", "RArm"]

    dy = 0.06
    dz = 0.06
    pathList     = [
                    [
                     [0.0, +dy, -dz, 0.0, 0.0, 0.0], # target  1 for "Torso"
                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # target  2 for "Torso"
                     [0.0, -dy, -dz, 0.0, 0.0, 0.0], # target  3 for "Torso"
                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # target  4 for "Torso"
                     [0.0, +dy, -dz, 0.0, 0.0, 0.0], # target  5 for "Torso"
                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # target  6 for "Torso"
                     [0.0, -dy, -dz, 0.0, 0.0, 0.0], # target  7 for "Torso"
                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # target  8 for "Torso"
                     [0.0, +dy, -dz, 0.0, 0.0, 0.0], # target  9 for "Torso"
                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # target 10 for "Torso"
                     [0.0, -dy, -dz, 0.0, 0.0, 0.0], # target 11 for "Torso"
                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], # target 12 for "Torso"
                     ],
                     [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], # for "LArm"
                     [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], # for "LArm"
                    ]

    axisMaskList = [almath.AXIS_MASK_ALL, # for "Torso"
                    almath.AXIS_MASK_VEL, # for "LArm"
                    almath.AXIS_MASK_VEL] # for "RArm"

    coef       = 0.5
    timesList  = [
                  [coef*(i+1) for i in range(12)], # for "Torso" in seconds
                  [coef*12],                       # for "LArm" in seconds
                  [coef*12]                        # for "RArm" in seconds
                 ]

    isAbsolute   = False

    motionProxy.positionInterpolations(effectorList, space, pathList, 
                                      axisMaskList, timesList, isAbsolute)


    # Deactivate whole body
    isEnabled    = False
    motionProxy.wbEnable(isEnabled)

    # Send NAO to Pose Init
    postureProxy.goToPosture("StandInit", 0.5)
    
def dab():
        
        motionProxy.setStiffnesses("Body", 1.0)
        postureProxy.goToPosture("Crouch", 0.3)
    
        # Example showing how to set angles, using a fraction of max speed
        names  = ["Body"]
        angles  = [-0.6872739791870117, 0.43254613876342773, -0.7271580696105957, 0.8636000156402588, 2.0785279273986816, -0.03490658476948738, -1.8238691091537476, 0.7567999958992004, -0.2438640594482422, -0.07359004020690918, -0.6626460552215576, 2.112546443939209, -1.1842899322509766, 0.07520794868469238, -0.2438640594482422, 0.052197933197021484, -0.6765360832214355, 2.112546443939209, -1.1857401132583618, -0.08125996589660645, 0.18565607070922852, 0.3141592741012573, 0.5920820236206055, 1.282465934753418, -0.0844118595123291, 0.8459999561309814]
        fractionMaxSpeed  = 0.2
        motionProxy.setAngles(names, angles, fractionMaxSpeed)
    
        time.sleep(3.0)
        motionProxy.setStiffnesses("Body", 1.0)
        angles = [0.7224719524383545, 0.46782803535461426, -0.15804386138916016, -0.1565098762512207, -0.3237159252166748, -1.2609061002731323, 0.19937801361083984, 0.7555999755859375, -0.23619413375854492, -0.06131792068481445, -0.5675380229949951, 2.112546443939209, -1.182755947113037, 0.07827591896057129, -0.23619413375854492, 0.05373191833496094, -0.5722239017486572, 2.112546443939209, -1.1857401132583618, -0.0827939510345459, -0.8497941493988037, -0.7501680850982666, 0.9771161079406738, 0.07674193382263184, -1.7012481689453125, 0.7192000150680542]
    
        fractionMaxSpeed  = 0.2
        motionProxy.setAngles(names, angles, fractionMaxSpeed)
#        
#        self.motionProxy.setStiffnesses("Body", 1.0)
    
#        time.sleep(3.0)
#        self.motionProxy.setStiffnesses("Body", 1.0)
#        angles = [0.09966802597045898, 0.5092461109161377, 1.1565940380096436, 1.2839161157608032, -1.4742159843444824, -0.04597806930541992, 0.18250393867492676, 0.785599946975708, -0.2730100154876709, 0.21326804161071777, -0.8743381500244141, 2.112546443939209, -1.1842899322509766, 0.06753802299499512, -0.2730100154876709, 0.13810205459594727, -0.4418339729309082, 1.7349958419799805, -1.1136419773101807, 0.12582993507385254, -0.420274019241333, -1.115260124206543, 1.7533200979232788, 0.5093300342559814, -1.8238691091537476, 0.6979999542236328]
    
#        fractionMaxSpeed  = 0.1
#        self.motionProxy.setAngles(names, angles, fractionMaxSpeed)
#       
#        self.motionProxy.setStiffnesses("Body", 1.0)
    
        time.sleep(3.0)
        motionProxy.setStiffnesses("Body", 1.0)
        angles = [-0.5737578868865967, 0.35584592819213867, 1.992624044418335, 0.49697399139404297, -0.5584180355072021, -1.0798940658569336, -1.4051861763000488, 0.7087999582290649, -0.2638061046600342, -0.06592011451721191, -0.5782761573791504, 2.112546443939209, -1.185823917388916, 0.08594608306884766, -0.2638061046600342, 0.04912996292114258, -0.5890979766845703, 2.112546443939209, -1.1857401132583618, -0.0858621597290039, 0.3728039264678955, -0.5630199909210205, 1.6674160957336426, 1.432797908782959, -1.4849538803100586, 0.7588000297546387]
    
        fractionMaxSpeed  = 0.2
        motionProxy.setAngles(names, angles, fractionMaxSpeed)
        time.sleep(3.0)
        motionProxy.setStiffnesses("Body", 1.0) 
        fractionMaxSpeed  = 2
        for i in range(2):
        	motionProxy.closeHand('RHand')
        	time.sleep(0.05)
        	motionProxy.openHand('RHand')
        time.sleep(0.1)
        doInitialisation()
        
        
#==============================================================================
# les EVENTS  d un robot
#============================================================================== 

class HumanGreeterModule(ALModule):
    """ A simple module able to react
    to facedetection events

    """
    def __init__(self, name):
        ALModule.__init__(self, name)
        
        try:
            self.SpeechReco = ALProxy("ALSpeechRecognition",robotIP,port)
#            try :
#                print ' je tente d unsub'
#                self.SpeechReco.unsubscribe('Arr')
#                print 'a'
#                self.SpeechReco.unsubscribe('Arr')
#                print 'b'
#                #SpeechReco.unsubscribe('Arr')
##                print'c'
#            except :
#                pass
#                print 'Error was: ',e
        except Exception,e: 
            print"Could not create proxy to ALSpeechRecognition"
            print "Error was :",e

        try:
        #    print SpeechReco.isRunning()
            self.SpeechReco.setVocabulary(["Dab","Romain Du sceau d'eau pfr",'Romain Dussot',"Ca va","automorphisme","Bibi","Nao","Bonjour","chut","salut je m'appelle Romain","Allo","Fromage","Romain","Salut","Anticonstitutionnelle","Endomorphisme"],
                                          False)
        
        except Exception,e: 
            print"Could not set vocab."
            print "Error was :",e
            
        try : 
            self.SpeechReco.setVisualExpression(True)
            self.SpeechReco.setParameter("Sensitivity" , 0)
            
        except Exception,e:
            print "Error:", e
        # Subscribe to the FaceDetected event:
        self.mot = ''
        global memory
        memory = ALProxy("ALMemory")
#        memory.subscribeToEvent("FaceDetected",
#            "HumanGreeter",
#            "onFaceDetected")

#        memory.subscribeToEvent("robotHasFallen", "HumanGreeter",
#                                "RobotFall")

#        memory.subscribeToEvent("ALSoundLocalization/SoundLocated","HumanGreeter",
#                                "Soundalert")
        
    def Soundalert(self,idword,value,untruc):
        
        memory.unsubscribeToEvent("ALSoundLocalization/SoundLocated","HumanGreeter")
        A = memoryProxy.getData("ALSoundLocalization/SoundLocated")
        print(A)
        time.sleep(1)
        memory.subscribeToEvent("ALSoundLocalization/SoundLocated","HumanGreeter",
                                "Soundalert")
    def Subs(self):
        memory.subscribeToEvent("WordRecognized", "HumanGreeter",
                                "WordReco")
        
#    def RobotFall(self,*_args):
#        
#        memory.unsubscribeToEvent("robotHasFallen","HumanGreeter")
#        tts.say("Je suis tombé, aidez moi")
#        print 'je suis tombé'
#        memory.subscribeToEvent("robotHasFallen","HumanGreeter",
#                                "RobotFall")

    def WordReco(self,idword,value,untruc):
        
        
        memory.unsubscribeToEvent("WordRecognized","HumanGreeter")
#        SpeechReco.unsubscribe('python_client')
#        tts.enableNotifications()
#        print memory.getData("ALTextToSpeech/TextDone")
        print value
#        tts.say(str(value[0].split(',')))
        
        print "tu viens de dire "+str(value[0])
        self.mot = str(value[0])
        
#        SpeechReco.subscribe('python_client')
        memory.subscribeToEvent("WordRecognized", "HumanGreeter",
                                "WordReco")

        
        
#    def onFaceDetected(self, *_args):
#        """ This will be called each time a face is
#        detected.
#
#        """
# 
#        memory.unsubscribeToEvent("FaceDetected",
#            "HumanGreeter")
#
#        tts.say("Hello, you")
##        userArmArticular_r(motionProxy)
##        time.sleep(3)
##        userArmArticular(motionProxy)
##        time.sleep(3)
#
#        memory.subscribeToEvent("FaceDetected",
#            "HumanGreeter",
#            "onFaceDetected")
        
    def stop(self):
        try :
            memory.unsubscribeToEvent("FaceDetected","HumanGreeter")
        except:
            pass
        try:
            memory.unsubscribeToEvent("WordRecognized","HumanGreeter")
        except:
            pass
        try:
            memory.unsubscribeToEvent("ALSoundLocalization/SoundLocated","HumanGreeter")
        except:    
            pass
        
            
    def stopWord(self):
        memory.unsubscribeToEvent("WordRecognized","HumanGreeter")


#==============================================================================
# test generale pour un robot
#============================================================================== 

if __name__== "__main__":
    doInitialisation()
#    test de la vision du NAO
    try:
        parser = OptionParser()
        parser.add_option("--pip",
        help="Parent broker port. The IP address or your robot",
        dest="pip")
        parser.add_option("--pport",
        help="Parent broker port. The port NAOqi is listening to",
        dest="pport",
        type="int")
        parser.set_defaults(
        pip=robotIP,
        pport=9559)
    
        (opts, args_) = parser.parse_args()
        pip   = opts.pip
        
        #    print pip,pport
        pport = opts.pport
        # We need this broker to be able to construct
        # NAOqi modules and subscribe to other modules
        # The broker must stay alive until the program exists
        myBroker = ALBroker("myBroker",
           "0.0.0.0",   # listen to anyone
           0,           # find a free port and use it
           pip,         # parent broker IP
           pport)       # parent broker port
    
    
        # Warning: HumanGreeter must be a global variable
        # The name given to the constructor must be the name of the
        # variable
        print "define HumanGreeter"
        global HumanGreeter

        HumanGreeter = HumanGreeterModule("HumanGreeter")
        print "Ready"

#        A = memoryProxy.getData("ALSoundLocalization/SoundLocated")
#        distance,confidence =  5 , 0.15
#        tracker.registerTarget("Sound", [distance, confidence])
#        tracker.setMode('Move')
#        tracker.setRelativePosition([-0.5, 0.0, 0.0, 0.1, 0.1, 0.3])    
#        tracker.track("Sound")
#        time.sleep(60)
        
        
        
#        HumanGreeter.Subs()        
#        for i in range(50):
#            if HumanGreeter.mot != '':
#                HumanGreeter.stopWord()
#                if HumanGreeter.mot =='Bonjour':
#            
#                    tts.say('Bonjour vous')
#                    HumanGreeter.mot = ''
#                    
#                if HumanGreeter.mot =='Salut':
#                    
#                    tts.say('Yo')
#                    HumanGreeter.mot = ''
#                    
#                if HumanGreeter.mot == 'Ca va':
#                    
#                    tts.say('Tranquille')
#                    HumanGreeter.mot = ''
#                    print HumanGreeter.mot
#                    
#                if HumanGreeter.mot == 'Dab':
#                    
#                    dab()
#                    HumanGreeter.mot = ''
#                if HumanGreeter.mot == 'Endomorphisme' or HumanGreeter.mot == automorphisme:
#                    tts.say("C'est fini la prépa, boloss")
#                    HumanGreeter.mot = ''
#                
#                if HumanGreeter.mot == 'Dance':
#                    pass
#                    
#                HumanGreeter.Subs()
#                    
#            time.sleep(0.25)
        steps()
        time.sleep(2)
        
        dancer()
        time.sleep(2)
        
        while not memory.getData("ALTextToSpeech/TextDone")  :    
            print "Waiting to stop"
            time.sleep(0.1)
        print "Fin video..."
        HumanGreeter.stop()
        doStop()




              
        HumanGreeter.stop()
        myBroker.shutdown()
    except KeyboardInterrupt:
        pass
        HumanGreeter.stop()
        myBroker.shutdown()
    tracker.stopTracker()
    tracker.unregisterAllTargets()
      
    doStop()
