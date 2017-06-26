import sys
from naoqi import ALProxy
import time

def rest():

	robotIp = "deltanao"
	#robotIp = "172.20.26.29"	
	robotIp = "localhost"
	robotPort = 9559
	#robotPort = 9595



	if len(sys.argv) <= 1:
    		print "Usage python rest.py robotIP (optional default: 127.0.0.1)"
	else:
   		robotIp = sys.argv[1]

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

