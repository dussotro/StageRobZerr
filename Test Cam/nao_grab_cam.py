import numpy as np
import cv2
from PIL import Image
import sys, time, math
sys.path.append("pynaoqi-python2.7-2.1.4.13-linux64")
from naoqi import ALProxy
import rest
import camRepair
import time
import pylab as pl
import almath

# NAO parameters
<<<<<<< HEAD
robotIP, port = "172.20.25.150", 9559
=======
<<<<<<< HEAD
robotIP, port = "172.20.11.237", 9559
=======
robotIP, port = "172.20.13.63", 9559
>>>>>>> 831cfa681d613346052bc7213e2b5b876c57e2e1
>>>>>>> 0ee8a54027254e46d10507de394bc6f36b1aa046
ALMEMORY_KEY_NAMES = ["Device/SubDeviceList/HeadYaw/Position/Sensor/Value", "Device/SubDeviceList/HeadPitch/Position/Sensor/Value"]

# Detection threshold and counter parameters
temps = 200000

#seuil
seuilHaut = 700  
seuilBas = 180 
seuilSecu = 20000 #il ne detecte rien donc prend l'ecran entier comme objet 

#range de filtrage
HSVmin = np.array([0,100,50], np.uint8)
HSVmax = np.array([100,255,160], np.uint8)


# Movement parameters
bangbang = 0.7
inv_coeff_x, inv_coeff_y = -3.0, 3.0

def StiffnessOn(proxy, ok):
    pNames = "Body"
    pStiffnessLists = 1.0 if ok else 0.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

def recordData():
    memory = ALProxy("ALMemory", robotIP, port)
    data = list()
    for key in ALMEMORY_KEY_NAMES:
        value = memory.getData(key)
        data.append(value)
    return data

# LEDs parameters
def setLeds(R, G, B):
    ledsProxy = ALProxy("ALLeds", robotIP, port)
    ledsProxy.setIntensity('AllLedsRed', float(R) / 255.0)
    ledsProxy.setIntensity('AllLedsGreen', float(G) / 255.0)
    ledsProxy.setIntensity('AllLedsBlue', float(B) / 255.0)


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
    tts.setLanguage("English")
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

try:
    VideoProxy = ALProxy("ALVideoDevice",robotIP, port)
except Exception, e:
    print "Could not create proxy to AlVideoDevice"
    print "Error was: ", e
    

#Initialisation et parametrage "Motion" 


#Parametrage Camera
VideoProxy.setParam(18, 0) # "kCameraSelectID", 0 : camera top, 1 : camera bottom
resolution = 0  # 0 : QQVGA, 1 : QVGA, 2 : VGA
colorSpace = 11 # RGB

try :
    videoClient = VideoProxy.subscribe("python_client", resolution, colorSpace, 5)
except RuntimeError:
    camRepair.rep(robotIP, port)
    videoClient = VideoProxy.subscribe("python_client", resolution, colorSpace, 5)

from_video = VideoProxy.getImageRemote(videoClient)
imageWidth, imageHeight = from_video[0], from_video[1]
Ang_x, Ang_y = 61.0 ,48.0
midAng_x, midAng_y = Ang_x/2, Ang_y/2


# Speaker
tts = ALProxy("ALTextToSpeech", robotIP, port)

def Main():
    StiffnessOn(motionProxy, True)
    postureProxy.goToPosture("Crouch", 1.0)
    isRuning = True
    motionProxy.setWalkArmsEnabled(True, True)
    motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])
    img_PIL1, img_PIL2, img_PIL3, img_PIL4 = np.zeros((imageWidth, imageHeight)), np.zeros((imageWidth, imageHeight)), np.zeros((imageWidth, imageHeight)), np.zeros((imageWidth, imageHeight))
    pl.figure()
    pl.subplot(221)
    pl.imshow(img_PIL1)
    pl.subplot(222)
    pl.imshow(img_PIL2)
    pl.subplot(223)
    pl.imshow(img_PIL3)
    pl.subplot(224)
    pl.imshow(img_PIL4)
    
    while isRuning: 
        filein=open("cmd","r")
        cmd=filein.readline().split()[0]
        filein.close()
        t0 = time.time()
        pl.ion()
        
        if (cmd=="run"):
            # print "runing"
            if not(isRuning) :
                motionProxy.moveInit()
                StiffnessOn(motionProxy, True)
                isRuning=True
            
            # Grab the image (it is not a BGR, but a RGB)
            from_video = VideoProxy.getImageRemote(videoClient)
            img_nao = from_video[6]
            
            pl.figure()
            
            img_PIL1 = Image.frombytes("RGB", (imageWidth, imageHeight), img_nao)
#            pl.figure(figsize=(4,4))
#            pl.imshow(img_PIL)
#            pl.colorbar()
#            pl.savefig('Image/img_PIL1.png')
            pl.draw()
            
            
<<<<<<< HEAD
            img_PIL2 = cv2.cvtColor(np.asarray(img_PIL1), cv2.COLOR_RGB2HSV)
#            pl.figure(figsize=(4,4))
#            pl.imshow(img_PIL)
#            pl.colorbar()
#            pl.savefig('Image/img_PIL2.png')
            pl.draw()            

            
            img_PIL3 = cv2.blur(img_PIL2, (5,5))
#            pl.figure(figsize=(4,4))
#            pl.imshow(img_PIL)
#            pl.colorbar()
#            pl.savefig('Image/img_PIL3.png')
            pl.draw()

#            hue = cv2.split(img_PIL)[0]
#            sat = cv2.split(img_PIL)[1] 
#            val = cv2.split(img_PIL)[2]
#            print 'Done'
#            
#            pl.figure(figsize=(15,4))
#            pl.subplot(131)
#            pl.imshow(hue, cmap='Greys')
#            pl.colorbar()
#            pl.subplot(132)
#            pl.imshow(sat, cmap='Greys')
#            pl.colorbar()            
#            pl.subplot(133)
#            pl.imshow(val, cmap='Greys')
#            pl.colorbar()   
#            
#            pl.savefig('Image/HSV.png')
=======
            pl.figure(figsize=(12,4))
            pl.subplot(131)
            pl.imshow(hue, cmap='Greys')
            pl.subplot(132)
            pl.imshow(sat, cmap='Greys')            
            pl.subplot(133)
            pl.imshow(val, cmap='Greys')
                        
            pl.savefig('Image/HSV.png')
            
            seuil = 100.0
            ret, seg_sat = cv2.threshold(sat, seuil, 255.0, cv2.THRESH_BINARY)
            pl.figure(figsize=(4,4))
            pl.imshow(seg_sat)
            pl.savefig('Image/IsolementBalle.png')
            time.sleep(3)
>>>>>>> 0ee8a54027254e46d10507de394bc6f36b1aa046

            
            img_PIL4 = cv2.inRange(img_PIL3, HSVmin, HSVmax)
#            pl.figure(figsize=(4,4)) 
#            pl.imshow(img_PIL4)
#            pl.colorbar()
#            pl.savefig('Image/img_PIL4.png')
            pl.draw()
            
            
            contours, hierarchy = cv2.findContours(img_PIL4, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#            out = np.zeros_like(seg_sat)
#            cv2.drawContours(out, contours, -1, 255, 3)
#            cv2.imshow('drawContours', out)
#            cv2.waitKey(0)
#            cv2.destroyAllWindows()

            # The Chosen One
            best_area = 0
            omega = 0.0
            vitesse = 0.0
            for c in contours:
                area = cv2.contourArea(c)
                if area > best_area: best_area, best_c = area, c
            print "aire trouvee : ", best_area    
            
            if best_area > 0:
                # Centroid of The Chosen One
                M = cv2.moments(best_c)
                cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
                #transformation de "pixel" a angles
                cx = cx * Ang_x/imageWidth
                cy = cy * Ang_y/imageHeight
                #calcul de la difference d'angles entre le barycentre et le robot
                dx = cx - midAng_x
                dy = cy - midAng_y
    
                # NAO jobs
                x, y = recordData()
                x, y = x + dx, y + dy
                omega = x
<<<<<<< HEAD
                print " | dx : ", dx, " | dy : ", dy
=======
                print "aire : ", best_area, " | dx : ", dx, " | dy : ", dy
>>>>>>> 0ee8a54027254e46d10507de394bc6f36b1aa046
                motionProxy.setAngles("HeadYaw", -x*almath.TO_RAD, 0.2)
                motionProxy.setAngles("HeadPitch", y*almath.TO_RAD, 0.2)
            
            #theta egal a l'angle de rotation necessaire de la tete
            #on suppose qu'on voudrait y arriver en 1 seconde
            #donc la norme de theta egale la norme de x
            if omega < -0.5 : omega = -0.5
            if omega > +0.5 : omega = +0.5
            #on ajoute un maximum a omega pour garder l'equilibre
            #le maximum correspond au decalage maximum en une seconde (il ne peut pas aller plus loin en agnle que ce qu'il voit)
            if best_area > seuilHaut:
                vitesse = -bangbang
                setLeds(255, 0, 0)
            elif best_area < seuilBas :
                vitesse = +bangbang
                setLeds(0, 0, 255)
            elif best_area > seuilSecu:
                vitesse = 0.0
                setLeds(200,200,200)
            else:
                setLeds(0, 255, 0)
                vitesse = 0.0
            if omega > -0.4 and omega < +0.4: omega = 0.0
                
            print 'vitesse et theta', vitesse, omega    
            #motionProxy.post.setWalkTargetVelocity(vitesse, 0.0, omega, 0.3)
            
#            filein=open("cmd","w")
#            filein.write("stop")
#            filein.close() 
            
<<<<<<< HEAD
            time.sleep(0.2)
=======
            time.sleep(5)
>>>>>>> 0ee8a54027254e46d10507de394bc6f36b1aa046
            
        elif cmd=="stop":
            motionProxy.stopMove()
            camRepair.rep(robotIP, port)
            rest.rest(robotIP, port)
           
            isRuning=False
            filein=open("cmd","w")
            filein.write("idle")
            filein.close()        
    
        else:
            break
            pl.ioff()
            #pass
        
        print 'la commande etait : ', cmd
        print 'executer en : ', time.time() - t0
    
    
    
def Cancel(): 
    VideoProxy.unsubscribe(videoClient)
    postureProxy.goToPosture("Crouch", 0.8)
    StiffnessOn(motionProxy, False)


if __name__ == '__main__':
  
    Main()
        
    filein=open("cmd","w")
    filein.write("run")
    filein.close() 
        
        
