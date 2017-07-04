# -*- encoding: UTF-8 -*-
import time
import sys
from naoqi import ALProxy, ALModule, ALBroker
import motion
import select
import vision_showimages as vis
import numpy as np
import almath
from PyQt4.QtGui import QWidget, QImage, QApplication, QPainter, QPushButton
import signal
from optparse import OptionParser


#robotIP = "172.20.28.103" #Eta
robotIP = "172.20.13.63" #Rouge
#robotIP = "172.20.28.103" #Bleu
#robotIP = "172.20.11.237"# gamma 
#robotIP = "172.20.11.242"# beta

port = 9559
CameraID = 0
Frequency = 0.0 #low speed
t=1.0



try:
    SpeechReco = ALProxy("ALSpeechRecognition",robotIP,port)
    try :
        print ' je tente d unsub'
        SpeechReco.unsubscribe('Arr')
        print 'a'
        SpeechReco.unsubscribe('Arr')
        print 'b'
        SpeechReco.unsubscribe('Arr')
        print'c'
    except :
        pass
#        print 'Error was: ',e
except Exception,e: 
    print"Could not create proxy to ALSpeechRecognition"
    print "Error was :",e

try:
    print SpeechReco.getLanguage()
    SpeechReco.setVocabulary(["Bibi","Nao","Bonjour","chut","salut je m'appelle Romain","Allo","Fromage","Romain","Salut","Anticonstitutionnelle","Endomorphisme"],True)

except Exception,e: 
    print"Could not set vocab."
    print "Error was :",e

