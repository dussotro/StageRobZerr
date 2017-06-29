import sys
from naoqi import ALProxy
import time

def rest(IP, PORT):
	robotIp, robotPort = IP, PORT


	if len(sys.argv) <= 1:
    		print "Usage python rest.py robotIP (optional default: 127.0.0.1)"
	else:
   		robotIp = IP

	try:
    		motionProxy = ALProxy("ALMotion", robotIp, robotPort)
	except Exception, e:
    		print "Could not create proxy to ALMotion"
    		print "Error was: ", e

	try:
    		postureProxy = ALProxy("ALRobotPosture", robotIp, robotPort)
	except Exception, e:
    		print "Could not create proxy to ALRobotPosture"
    		print "Error was: ", e

		jointName = "Body"

	# new feature in 1.14.1
	motionProxy.wakeUp()

	postureProxy.goToPosture("Crouch", 1.0)

	# Putting NAO at rest position (no stiffness, to save power)
	motionProxy.rest()
