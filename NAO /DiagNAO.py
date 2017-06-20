# -*- coding: utf-8 -*-
import time
import sys
from naoqi import ALProxy, ALModule
import motion
import select
import vision_definitions
import numpy as np
import almath

print 2

robotIP = "172.20.12.126"
#robotIP = "172.20.28.103"
port = 9559
Frequency = 0.0 #low speed
t=1

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



#stiffness for real NAO Robot
def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

def doInitialisation():
    print">>>>>> Initialisation"
    # Set NAO in Stiffness On
    StiffnessOn(motionProxy)
    # Send NAO to Pose Init
    postureProxy.goToPosture("StandInit", 0.5)
    

#class Battery(ALModule):
#    """ Mandatory docstring.
#        comment needed to create a new python module
#    """
#    def __init__(self, name):
#        ALModule.__init__(self, name)
#        # No need for IP and port here because
#        # we have our Python broker connected to NAOqi broker
#        # Create a proxy to ALTextToSpeech for later use
#        self.battery = BatteryProxy
#        self.level = 0
#
#        # Subscribe to the BatteryChange event:
#        self.battery.subscribeToEvent("BatteryChargeChanged",
#            "BatteryRob",
#            "callBackBattery")
#        
#  def callBackBattery(self, *_args):
#    """ Mandatory docstring.
#        comment needed to create a bound method
#    """
#    self.battery.unsubscribeToEvent("BatteryChargeChanged",
#            "BatteryRob") 
#    
#    self.level = self.
#    pass

#==============================================================================
# Classe de test de toutes les articulations
#==============================================================================


class Robot:
	def __init__(self, rt, ia, am, space):
            self.d_mvt = {}
            self.reference_time = rt
            self.isAbsolute = ia
            self.axisMask = am
            self.space = space
            self.tempo_time = 0

	def mvt(self, where, path):
            reference_time = 0
            if self.tempo_time != 0:
            	reference_time = self.tempo_time
            	self.tempo_time = 0
            else:
            	reference_time = self.reference_time	
            if not self.d_mvt.has_key(where):
            	self.d_mvt[where] = path
            else:
            	old_path = self.d_mvt[where]
            	saved_path = [el1 + el2 for el1, el2 in zip(path, old_path)]
            	self.d_mvt[where] = saved_path
            motionProxy.positionInterpolation(where, self.space, path, self.axisMask, reference_time, self.isAbsolute)
                    
            time.sleep(1.0)



#==============================================================================
# Audio
#==============================================================================
def TestTts(texte):
    tts.say(texte)
	
#==============================================================================
# """Vision"""
#==============================================================================


def Test_Image():

	####
	# Create proxy on ALVideoDevice

	print "Creating ALVideoDevice proxy to ", robotIP

	camProxy = ALProxy("ALVideoDevice", robotIP, port)

	####
	# Register a Generic Video Module

	resolution = vision_definitions.kQVGA
	colorSpace = vision_definitions.kYUVColorSpace
	fps = 30

	nameId = camProxy.subscribe("python_GVM", resolution, colorSpace, fps)
	print nameId

	print 'getting images in local'
	for i in range(0, 20):
	  camProxy.getImageLocal(nameId)
	  camProxy.releaseImage(nameId)

	resolution = vision_definitions.kQQVGA
	camProxy.setResolution(nameId, resolution)

	print 'getting images in remote'
	for i in range(0, 20):
	  camProxy.getImageRemote(nameId)

	camProxy.unsubscribe(nameId)

	print 'end of gvm_getImageLocal python script'

def showNaoImage():
    videoRecorderProxy = ALProxy("ALVideoRecorder", robotIP, port)
    
    # This records a 320*240 MJPG video at 10 fps.
    # Note MJPG can't be recorded with a framerate lower than 3 fps.
    videoRecorderProxy.setResolution(1)
    videoRecorderProxy.setFrameRate(10)
    videoRecorderProxy.setVideoFormat("MJPG")
    videoRecorderProxy.startRecording("./", "test")

    time.sleep(5)
    # Video file is saved on the robot in the
    # /home/nao/recordings/cameras/ folder.
    videoInfo = videoRecorderProxy.stopRecording()
    #print type video
    print "Video was saved on the robot: ", videoInfo[1]
    print "Num frames: ", videoInfo[0]
    video = memoryProxy.getData("./test.avi")


#==============================================================================
# """Sensors"""
#==============================================================================
def TrySensors():
 
    
    Left = memoryProxy.getData("Device/SubDeviceList/US/Left/Sensor/Value")
    Right = memoryProxy.getData("Device/SubDeviceList/US/Right/Sensor/Value") 
    print 'Left :', Left
    print 'Right:', Right
    

def Accelero():
    X = memoryProxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value")
    Y = memoryProxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value") 
    Z = memoryProxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value")
    print "X = ", X
    print "Y = ", Y
    print "Z = ", Z
    
    AngleX = memoryProxy.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value")
    AngleY = memoryProxy.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value")
    print "AngleX =" , AngleX
    print "AngleY =" , AngleY
    
#==============================================================================
# """Motion"""
#==============================================================================
def dorun(t):
    motionProxy.setWalkTargetVelocity(0.4, 0, 0, 0.8)
    motionProxy.moveTo (0.4, 0, 0)
#    time.sleep(t)
    print"running"
    

def doback():
    
     motionProxy.moveTo (-0.4, 0, 0)
     
     time.sleep(t)
     print"back"
    
def doleft(angle):
    
    theta= angle
    motionProxy.setWalkTargetVelocity(0, 0, 0.5, 0.01)
    motionProxy.moveTo (0, 0, theta)

#    time.sleep(t)
#    print"turning left"

def doright(angle):
    
    theta= -angle
#    motionProxy.moveTo (0, 0, theta)
    motionProxy.setWalkTargetVelocity(0, 0, -0.5, 0.01)
    time.sleep(t)
    print"turning right"
    
def doStandUp():
    
    motionProxy.wakeUp()
    motionProxy.setStiffnesses("Body", 1.0)
    time.sleep(t)
    print"standing up"
    
def doStop():
    

    postureProxy.goToPosture("Crouch", 0.3)
    motionProxy.setStiffnesses("Body", 0.0)
    motionProxy.rest()
    time.sleep(t)
    print"Stopping"


def target_velocity():
    #TARGET VELOCITY
    X = 0.4
    Y = 0.0
    Theta = 0.0
    Frequency =1.0 # max speed
    motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)

    time.sleep(4.0)
    print "walk Speed X :",motionProxy.getRobotVelocity()[0]," m/s"
    
    X = -0.4  #backward
    Y = 0.0
    Theta = 0.0
    Frequency =0.0 # low speed
    motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)
    
    time.sleep(4.0)
    print "walk Speed X :",motionProxy.getRobotVelocity()[0]," m/s"

def position_robot():
    
    initRobotPosition = m.Pose2D(motionProxy.getRobotPosition(False))

    X = 0.3
    Y = 0.1
    Theta = np.pi/2.0
    motionProxy.post.moveTo(X, Y, Theta)
    # wait is useful because with post moveTo is not blocking function
    motionProxy.waitUntilMoveIsFinished()

    #####################
    ## get robot position after move
    #####################
    endRobotPosition = m.Pose2D(motionProxy.getRobotPosition(False))

    #####################
    ## compute and print the robot motion
    #####################
    robotMove = m.pose2DInverse(initRobotPosition)*endRobotPosition
    print "Robot Move :", robotMove

def Test_Square_Left_Right():
    err = 0.2
    print ">>>>>>>>>>> Test du carre"
    print ">>> carre gauche"
    for i in range(4):
        dorun(1)
        doleft(np.pi/2)
    print ">>> carre droite"
    for j in range(4):
        dorun(1)
        doright(np.pi/2)
    print "fin de test du carre"


def Battery():
    percentage = memoryProxy.getData("Device/SubDeviceList/Battery/Current/Sensor/Value") 
    b = memoryProxy.getData ("Device/SubDeviceList/Battery/Charge/Sensor/Status")
    c = memoryProxy.getData ("Device/SubDeviceList/Battery/Charge/Sensor/Value")
    print"percentage = ", percentage
    print "b =", b
    print "c =", c

def Test_Articulations():
    StiffnessOn(motionProxy)

    # Send NAO to Pose Init
    postureProxy.goToPosture("StandInit", 1.0)

    space      = motion.FRAME_ROBOT
    axisMask   = almath.AXIS_MASK_ALL   # full control
    isAbsolute = False

    robot = Robot(1.0, isAbsolute, axisMask, space)
    
    #===
    BodyPart = "RArm"
    print BodyPart
    print " Roll"
    robot.mvt(BodyPart, [0.0, 0.0, 0.0,-0.31, 0.0, 0.0])
    robot.mvt(BodyPart, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    robot.mvt(BodyPart, [0.0, -0.5, 0.0, 0.0, 0.0, 0.0])
    
    
    #===
    
    postureProxy.goToPosture("Crouch", 1.0)

#
#def shoot():
#    
#     # Activate Whole Body Balancer
#    isEnabled  = True
#    proxy.wbEnable(isEnabled)
#
#    # Legs are constrained fixed
#    stateName  = "Fixed"
#    supportLeg = "Legs"
#    proxy.wbFootState(stateName, supportLeg)
#
#    # Constraint Balance Motion
#    isEnable   = True
#    supportLeg = "Legs"
#    proxy.wbEnableBalanceConstraint(isEnable, supportLeg)
#
#    # Com go to LLeg
#    supportLeg = "LLeg"
#    duration   = 2.0
#    proxy.wbGoToBalance(supportLeg, duration)
#
#    # RLeg is free
#    stateName  = "Free"
#    supportLeg = "RLeg"
#    proxy.wbFootState(stateName, supportLeg)
#
#    # RLeg is optimized
#    effectorName = "RLeg"
#    axisMask     = 63
#    space        = motion.FRAME_ROBOT
#
#
#    # Motion of the RLeg
#    dx      = 0.05                 # translation axis X (meters)
#    dz      = 0.05                 # translation axis Z (meters)
#    dwy     = 5.0*math.pi/180.0    # rotation axis Y (radian)
#
#
#    times   = [2.0, 2.7, 4.5]
#    isAbsolute = False
#
#    targetList = [
#      [-dx, 0.0, dz, 0.0, +dwy, 0.0],
#      [+dx, 0.0, dz, 0.0, 0.0, 0.0],
#      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
#
#    proxy.positionInterpolation(effectorName, space, targetList,
#                                 axisMask, times, isAbsolute)
#
#
#    # Example showing how to Enable Effector Control as an Optimization
#    isActive     = False
#    proxy.wbEnableEffectorOptimization(effectorName, isActive)
#
#    # Com go to LLeg
#    supportLeg = "RLeg"
#    duration   = 2.0
#    proxy.wbGoToBalance(supportLeg, duration)
#
#    # RLeg is free
#    stateName  = "Free"
#    supportLeg = "LLeg"
#    proxy.wbFootState(stateName, supportLeg)
#
#    effectorName = "LLeg"
#    proxy.positionInterpolation(effectorName, space, targetList,
#                                axisMask, times, isAbsolute)
#
#    time.sleep(1.0)
#
#    
if __name__== "__main__":
    doInitialisation()
    #test de la vision du NAO
    try:
        #Test_Detection()
        #Test_Image()
        #test de capteurs 
        #TrySensors()
        #target_velocity()
    
        #showNaoImage()
        TestTts("michel est mort ce soir")


#        #test de déplacements
#        dorun(1)
#        doback()
#        doleft()
#        doright()
#        doStandUp()
        Test_Articulations()
    except Exception, e:
        print'erreur: ', e
    print "michel"    
    doStop()
