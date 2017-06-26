from naoqi import ALProxy

def rep():
	camProxy=ALProxy("ALVideoDevice","127.0.0.1",9559)
	camProxy.unsubscribe("python_client")

