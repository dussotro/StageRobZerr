
from PyQt4 import QtGui,uic
from naoqi import ALProxy, ALModule
import DiagNAO
import time
import sys
import os


robotIP = "172.20.27.244" #Rouge
#robotIP = "172.20.28.103" #Bleu
#robotIP = "172.20.11.237"# gamma 

port = 9559
CameraID = 0
Frequency = 0.0 #low speed
t=1.0

#try:
#    motionProxy = ALProxy("ALMotion", robotIP, port)
#except Exception, e:
#    print"Could not create proxy to ALMotion"
#    print"Error was: ", e
#try:
#    postureProxy = ALProxy("ALRobotPosture", robotIP, port)
#except Exception, e:
#    print "Could not create proxy to ALRobotPosture"
#    print "Error was: ", e
#try:
#    sonarProxy = ALProxy("ALSonar", robotIP, port)
#    sonarProxy.subscribe("myApplication")
#except Exception, e:
#    print "Could not create proxy to ALSonar"
#    print "Error was: ", e
#
#try :
#    audio = ALProxy("ALAudioDevice", robotIP,port)
#    audio.setOutputVolume(50)
#except Exception, e: 
#    print "Could not create proxy to ALaudioProxy"
#    print "Error was: ", e
#try :
#    tts = ALProxy("ALTextToSpeech", robotIP, port)
#    tts.setLanguage("French")
#except Exception, e: 
#    print "Could not create proxy to ALTextToSpeech"
#    print "Error was: ", e
#
#try:
#    memoryProxy = ALProxy("ALMemory",robotIP, port)
#except Exception, e:
#    print "Could not create proxy to ALMemory"
#    print "Error was: ", e
#    
#try:
#    BatteryProxy = ALProxy("ALBattery",robotIP, port)
#except Exception, e:
#    print "Could not create proxy to AlBattery"
#    print "Error was: ", e


class UiTest(QtGui.QMainWindow):
    def __init__(self,*args, **kwargs):
        self.battery_init = -1
        QtGui.QMainWindow.__init__(self, *args, **kwargs)
        self.ui = uic.loadUi('Interface.ui', self)
        
        self.ui.Bouton_square.clicked.connect(self.Square)
        self.ui.Bouton_battery.clicked.connect(self.Battery)
        self.ui.Bouton_sensors.clicked.connect(self.Sensors)
        self.ui.Bouton_Stop.clicked.connect(self.Stop)
        DiagNAO.doInitialisation()
        self.battery_init = DiagNAO.BatteryMemory()
        
    def Square(self):
        DiagNAO.Test_Square_Left_Right()
        
    def Battery(self):
        battery_t = DiagNAO.BatteryMemory()
        self.ui.label_battery.setText(str(self.battery_init - battery_t) + '%')     
        
    def Sensors (self):
        self.ui.label_Gsens.setText(str(round(DiagNAO.TrySensors()[0],1)))
        self.ui.label_Dsens.setText(str(round(DiagNAO.TrySensors()[1],1)))
    
    def Stop(self):
        DiagNAO.doStop()
            
        
        
    
        
        
if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    window = UiTest()
    window.show()
    sys.exit(app.exec_())
    