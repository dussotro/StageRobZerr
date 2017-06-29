# -*- coding: utf-8 -*-
from PyQt4 import QtGui,uic
from naoqi import ALProxy, ALModule
from PyQt4.QtGui import QWidget, QImage, QApplication, QPainter
from vision_showimages import *
import vision_definitions
import DiagNAO
import time
import sys
import os
from multiprocessing import Process, Queue, Value
import signal


#robotIP = "172.20.13.63" #Rouge
#robotIP = "172.20.28.103" #Bleu
robotIP = "172.20.11.237"# gamma 
#robotIP = "172.20.11.242" #beta

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
        self.prog = Value('i',0)
        self.ui = uic.loadUi('Interface.ui', self)            
        
        self.ui.Bouton_square.clicked.connect(self.Square)
        self.ui.Bouton_battery.clicked.connect(self.Battery)
        self.ui.Bouton_sensors.clicked.connect(self.Sensors)
        self.ui.Bouton_Stop.clicked.connect(self.Stop)
        self.ui.Bouton_Tete.clicked.connect(self.Tete)
        self.ui.Bouton_Total.clicked.connect(self.Total)
        self.ui.Bouton_main.clicked.connect(self.Main)
        self.ui.Bouton_Epaule.clicked.connect(self.Epaule)
        self.ui.Bouton_Poignet.clicked.connect(self.Poignet)
        self.ui.Bouton_Coude.clicked.connect(self.Coude)
        self.ui.Bouton_Camera.clicked.connect(self.Camera)
    
        self.inage= ImageWidget(robotIP, port,0)
        
        DiagNAO.doInitialisation()
        self.battery_init = DiagNAO.BatteryMemory()
        
        palette= QtGui.QPalette()
        pixmap = QtGui.QPixmap("fond.jpeg").scaled(self.ui.width(),self.ui.height())   
        # vérifiez que vous avez cette image
        palette.setBrush(QtGui.QPalette.Background,QtGui.QBrush(pixmap))
        self.setPalette(palette)
        
        
        
#    def setprog(self,signal,signale):
#        print signal,signale        
#        self.prog = 0
#        print "setter {}".format(self.prog)
#        print self
#            
    def Square(self): 
        if self.prog.value == 0:
            self.prog.value = 1 
            q = Queue()
            p = Process(target=DiagNAO.Test_Square_Left_Right(), args=(q,self.prog))
            p.start()
        
        
    def Battery(self):
        if self.prog.value == 0:
            self.ui.label_prog.setText('Battery')
            self.prog = 1
            battery_t = DiagNAO.BatteryMemory()
            self.ui.label_battery.setText(str(battery_t) + '%')
            self.ui.label_prog.setText('')
            self.prog.value = 0
            
        
    def Sensors (self):
        if self.prog.value == 0:
            self.prog.value = 1
            self.ui.label_prog.setText('Sensors')
            self.ui.label_Gsens.setText(str(round(DiagNAO.TrySensors()[0],1)))
            self.ui.label_Dsens.setText(str(round(DiagNAO.TrySensors()[1],1)))
            self.ui.label_prog.setText('')
            self.prog.value = 0
            
    def Tete(self):
        if self.prog.value == 0:
            self.prog.value = 1
            q = Queue()
            p = Process(target=DiagNAO.Tete, args=(q,self.prog))
            p.start()
        
    def Total(self):
        if self.prog.value == 0:
            self.prog.value = 1
            q = Queue()
            p = Process(target  = DiagNAO.Test_Articulations, args=(q,self.prog))
            p.start()
    
    def Stop(self):
        DiagNAO.doStop()        
        
    def Main(self):
        if self.prog.value == 0:
            self.prog.value = 1 
            q = Queue()
            p = Process(target=DiagNAO.Main, args=(q,self.prog))
            p.start()
            
    def Epaule(self):
        if self.prog.value == 0:
            self.prog.value = 1 
            q = Queue()
            p = Process(target=DiagNAO.Epaule, args=(q,self.prog))
            p.start()
        
    
    def Coude(self):
        if self.prog.value == 0:
            self.prog.value = 1
            self.ui.label_prog.setText('Coudes')
            DiagNAO.Coudes()
            self.ui.label_prog.setText('')  
            self.prog =0
    
    def Poignet(self):
        if self.prog.value == 0:
            self.prog.value = 1
            self.ui.label_prog.setText('Poignet')
            DiagNAO.Poignet()
            self.ui.label_prog.setText('')
            self.prog.value = 0
    
    def Camera(self):
        if self.inage.isHidden() :
            self.inage.show()
            
    def closeEvent(self,event):
        DiagNAO.doStop()
        event.accept()        
    
        
        
        
    
        
        
if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    window = UiTest()
    window.show()
    sys.exit(app.exec_())
    