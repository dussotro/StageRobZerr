from naoqi import ALProxy

def rep(IP, PORT):
	camProxy=ALProxy("ALVideoDevice",IP, PORT)
	camProxy.unsubscribe("python_client")

