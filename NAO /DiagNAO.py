import time
import sys
from naoqi import ALProxy
import motion
import select
import vision_definitions

robotIP = "172.20.12.126"
port = 9559
Frequency = 0.0 #low speed

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
except Exception, e:
    print "Could not create proxy to ALSonar"
    print "Error was: ", e


try:
    memoryProxy = ALProxy("ALMemory",robotIP, port)
except Exception, e:
    print "Could not create proxy to ALMemory"
    print "Error was: ", e
           
#stiffness for real NAO Robot
def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

def doInitialisation():
    print(">>>>>> Initialisation")
    # Set NAO in Stiffness On
    StiffnessOn(motionProxy)
    # Send NAO to Pose Init
    postureProxy.goToPosture("StandInit", 0.5)

	
#==============================================================================
# """Vision"""
#==============================================================================

def Test_Detection():

    # Create a proxy to ALFaceDetection
    try:
        faceProxy = ALProxy("ALFaceDetection", robotIP, port)
    except Exception, e:
        print "Error when creating face detection proxy:"
        print str(e)
        exit(1) 


    period = 500
    faceProxy.subscribe("Test_Face", period, 0.0 )

    # A simple loop that reads the memValue and checks whether faces are detected.
    for i in range(0, 20):
        time.sleep(0.5)
        val = memoryProxy.getData(memValue)

        print ""
        print "*****"
        print ""

        # Check whether we got a valid output.
        if(val and isinstance(val, list) and len(val) >= 2):

            # We detected faces !
            # For each face, we can read its shape info and ID.

            # First Field = TimeStamp.
            timeStamp = val[0]

            # Second Field = array of face_Info's.
            faceInfoArray = val[1]

            try:
                # Browse the faceInfoArray to get info on each detected face.
                for j in range( len(faceInfoArray)-1 ):
                    faceInfo = faceInfoArray[j]
                    # First Field = Shape info.
                    faceShapeInfo = faceInfo[0]

                    # Second Field = Extra info (empty for now).
                    faceExtraInfo = faceInfo[1]

                print "  alpha %.3f - beta %.3f" % (faceShapeInfo[1], faceShapeInfo[2])
                print "  width %.3f - height %.3f" % (faceShapeInfo[3], faceShapeInfo[4])
 
            except Exception, e:
                print "faces detected, but it seems getData is invalid. ALValue ="
                print val
                print "Error msg %s" % (str(e))
        else:
              print "No face detected"

        # Unsubscribe the module.
        faceProxy.unsubscribe("Test_Face")

        print "Detection finished"

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


#==============================================================================
# """Sensors"""
#==============================================================================
def TrySensors():
    Left = memoryProxy.getData("Device/SubDeviceList/US/Left/Sensor/Value")
    Right = memoryProxy.getData("Device/SubDeviceList/US/Right/Sensor/Value") 
    print 'Left :', Left
    print 'Right:', Right
    

#==============================================================================
# """Motion"""
#==============================================================================
def dorun():
    
    motionProxy.moveTo (0.8, 0, 0)
    sleep(1.0)
    print"running"
    

def doback():
    
     motionProxy.moveTo (-0.8, 0, 0)
     sleep(1.0)
     print"back"
    
def doleft():
    
    theta= -(np.pi/6)
    motionProxy.moveTo (0, 0, theta)
    sleep(1.0)
    print"turning left"

def doright():
    
    theta= (np.pi/6)
    motionProxy.moveTo (0, 0, theta)
    sleep(1.0)
    print"turning right"
    
def doStandUp():
    
    motionProxy.wakeUp()
    motionProxy.setStiffnesses("Body", 1.0)
    sleep(1.0)
    print"standing up"
    
def doStop():
    
    motionProxy.rest()
    motionProxy.setStiffnesses("Body", 0.0)
    sleep(deltat)
    print"stoping"

    
if __name__== "__main__":
    doInitialisation()
    #test de la vision du NAO
    #Test_Detection()
    
    #test de capteurs 
    TrySensors()
    
    #test de déplacements
    dorun()
    doback()
    doleft()
    doright()
    doStandUp()
    doStop()
    



