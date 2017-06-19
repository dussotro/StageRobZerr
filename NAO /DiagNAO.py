import time
import sys
from naoqi import ALProxy
import motion
import select

robotIP = "localhost"
port = 11212
Frequency = 0.0 #low speed

try:
    motionProxy = ALProxy("ALMotion", robotIP, port)
except Exception, e:
    print "Could not create proxy to ALMotion"
    print "Error was: ", e
try:
    postureProxy = ALProxy("ALRobotPosture", robotIP, port)
except Exception, e:
    print "Could not create proxy to ALRobotPosture"
    print "Error was: ", e




#stiffness for real NAO Robot
def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)


# functions (actions of the fsm)
def doInitialisation():
    print ">>>>>> Initialisation"   
    
    # Set NAO in Stiffness On
    StiffnessOn(motionProxy)
    # Send NAO to Pose Init
    postureProxy.goToPosture("StandInit", 0.5)
    return event # return event to be able to define the transition


if __name__== "__main__":
    




