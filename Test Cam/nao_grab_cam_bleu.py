# -*- encoding: UTF-8 -*- 

#--------------------------------------------------------------------------#
#  Bibliotheque                                                            #
#--------------------------------------------------------------------------#

import numpy as np
import cv2
from PIL import Image
import sys, time, math
sys.path.append("pynaoqi-python2.7-2.1.4.13-linux64")
from naoqi import ALProxy
import time
import pylab as pl
import almath

#--------------------------------------------------------------------------#
#  Constantes                                                              #
#--------------------------------------------------------------------------#

# NAO parameters
#robotIP, port = "172.20.27.47", 9559
#robotIP, port = "172.20.25.150", 9559
#robotIP, port = "172.20.25.150", 9559

#utilisation de sys
if len(sys.argv) >= 2:
    robotIP, port =  sys.argv[1], 9559
    
#On liste les futures valeurs de capteurs à récupérer
ALMEMORY_KEY_NAMES = ["Device/SubDeviceList/HeadYaw/Position/Sensor/Value", "Device/SubDeviceList/HeadPitch/Position/Sensor/Value"]

#Seuil de modification de comportement
seuilHaut = 1000  
seuilBas = 200
seuilSecu = 20000 #il ne detecte rien donc prend l'ecran entier comme objet (pas tout le temps) 

#On cree un choix d'affichage entier ou partiel
#1 pour entier (7 images)
#0 pour partiel (4 images)
affichage = 1


#On cree des limites de detection pour isoler un objet avec OpenCV

#pour la couleur orange
#HSVmin = np.array([0,100,120], np.uint8)
#HSVmax = np.array([40,240,255], np.uint8)

#pour la couleur bleu
HSVmin = np.array([90,100,0], np.uint8)
HSVmax = np.array([120,250,150], np.uint8)

# on definit une constante qui permettront de modifier les differentes vitesses de deplacements du robot
inv_coeff_x, inv_coeff_y = -3.0, 3.0

#on definit une fonction de "réparation" du prosy de la caméra (seulement un unsubscribe en fait...)
def rep(IP, PORT):
	camProxy=ALProxy("ALVideoDevice",IP, PORT)
	camProxy.unsubscribe("python_client")

#on definit la mise en place (ou non) de la stiffness du robot 
def StiffnessOn(proxy, ok):
    pNames = "Body"
    pStiffnessLists = 1.0 if ok else 0.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

#On recupere les valeurs de position de la tete (Pitch et Raw) grace a la constante du debut ALMEMORY_KEY_NAMES
def recordData():
    memory = ALProxy("ALMemory", robotIP, port)
    data = list()
    for key in ALMEMORY_KEY_NAMES:
        value = memory.getData(key)
        data.append(value)
    return data

#on definit un moyen de changer la couleur des leds du robot pour reperer son etat
def setLeds(R, G, B):
    ledsProxy = ALProxy("ALLeds", robotIP, port)
    ledsProxy.setIntensity('AllLedsRed', float(R) / 255.0)
    ledsProxy.setIntensity('AllLedsGreen', float(G) / 255.0)
    ledsProxy.setIntensity('AllLedsBlue', float(B) / 255.0)

#on definit une fonction de decoupage d'une image en 3 images, utiles pour les tests et la comprehension du traitement d'image
#pendant le fonctionnement, il n'est pas necessaire de la laisser
def splitImage(img, cpt):
    hue = cv2.split(img)[0]
    sat = cv2.split(img)[1] 
    val = cv2.split(img)[2]
    print 'Done'
    #affichage sur la meme figure que la mise a jour en continue
    pl.subplot(337)
    pl.imshow(hue, cmap='Greys')
    if not cpt : pl.colorbar()
    pl.subplot(338)
    pl.imshow(sat, cmap='Greys')
    if not cpt : pl.colorbar()            
    pl.subplot(339)
    pl.imshow(val, cmap='Greys')
    if not cpt : pl.colorbar()   
    
    pl.savefig('Image/HSV.png')

    

#--------------------------------------------------------------------------#
#  Phase d'initialisation                                                  #
#--------------------------------------------------------------------------#

#on cree les differents proxy du robot
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
    tts.setLanguage("French")
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
    

#Initialisation et parametrage du proxy "Motion" 


#Parametrage Camera
VideoProxy.setParam(18, 0) # "kCameraSelectID", 0 : camera top, 1 : camera bottom
resolution = 0  # 0 : QQVGA, 1 : QVGA, 2 : VGA
colorSpace = 11 # RGB

try :
    videoClient = VideoProxy.subscribe("python_client", resolution, colorSpace, 5)
except RuntimeError:
    camRepair.rep(robotIP, port)
    videoClient = VideoProxy.subscribe("python_client", resolution, colorSpace, 5)

#on definit les constantes de la video 
from_video = VideoProxy.getImageRemote(videoClient)
imageWidth, imageHeight = from_video[0], from_video[1]
#les angles represente les angles d'ouverture de la camera affiches sur l'ecran
Ang_x, Ang_y = 61.0 ,48.0
midAng_x, midAng_y = Ang_x/2, Ang_y/2

#--------------------------------------------------------------------------#
#  Main                                                                    #
#--------------------------------------------------------------------------#

def Main():
    #on initialise le robot
    StiffnessOn(motionProxy, True)
    postureProxy.goToPosture("Crouch", 1.0)
    isRuning = True
    motionProxy.setWalkArmsEnabled(True, True)
    motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])
    #on cree notre figure a rafraichir avec les images traitees par le NAO
    f = pl.figure()
    ax = f.gca()
    f.show()
    cpt = 0
        
    while isRuning:
        #on utilise un fichier cmd.py qui permet de modifier le comportement du robot en cours d'execution du programme 
        filein=open("cmd","r")
        cmd=filein.readline().split()[0]
        filein.close()
        t0 = time.time()
        #on initialise le mode interactif de matplotlib ou pylab
        pl.ion()
        
        if (cmd=="run"):
            #si le robot n'a pas ete initialise, on le fait 
            if not(isRuning) :
                motionProxy.moveInit()
                StiffnessOn(motionProxy, True)
                isRuning=True
            
            # on recupere la video du NAO
            from_video = VideoProxy.getImageRemote(videoClient)
            img_nao = from_video[6]
            
            #on transforme l'image du nao encode RGB en tableau de pixel
            if affichage:
                pl.subplot(331) 
            else:
                pl.subplot(221)
            img_PIL1 = Image.frombytes("RGB", (imageWidth, imageHeight), img_nao)
            pl.imshow(img_PIL1)
            if not cpt : pl.colorbar() #affichage de la bar une seule fois
#            pl.figure(figsize=(4,4))
#            pl.imshow(img_PIL)
#            pl.colorbar()
#            pl.savefig('Image/img_PIL1.png')
            

            #on applique une transformation de RGB a HSV pour eviter les problemes de luminosite par la suite
            if affichage:
                pl.subplot(333) 
            else:
                pl.subplot(222)            
            img_PIL2 = cv2.cvtColor(np.asarray(img_PIL1), cv2.COLOR_RGB2HSV)
            pl.imshow(img_PIL2)
            if not cpt : pl.colorbar()
#            pl.figure(figsize=(4,4))
#            pl.imshow(img_PIL)
#            pl.colorbar()
#            pl.savefig('Image/img_PIL2.png')


            #on floute l'image pour permettre de filtrer les erreurs de comprehension 
            #un pied de table eloigne mais d'une couleur similaire a celle detectee deviendra inperceptible
            if affichage:
                pl.subplot(334) 
            else:
                pl.subplot(223)
            img_PIL3 = cv2.blur(img_PIL2, (5,5))
            pl.imshow(img_PIL3)
            if not cpt : pl.colorbar()
#            pl.figure(figsize=(4,4))
#            pl.imshow(img_PIL)
#            pl.colorbar()
#            pl.savefig('Image/img_PIL3.png')
            
            #on filtre les couleurs que l'on veut garder dans l'image avec les bornes HSVmin et HSVmax
            if affichage:
                pl.subplot(336) 
            else:
                pl.subplot(224)
            img_PIL4 = cv2.inRange(img_PIL3, HSVmin, HSVmax)
            pl.imshow(img_PIL4)
            if not cpt : pl.colorbar()
#            pl.figure(figsize=(4,4)) 
#            pl.imshow(img_PIL4)
#            pl.colorbar()
#            pl.savefig('Image/img_PIL4.png')

            #le canvas permet de mettre a jour la meme figure facilement et afficher un "direct" (plutot leger differe)
            f.canvas.draw()
            
            
            #la fonction est definit plus haut et permet l'affichage du HSV sur 3 images
            if affichage :
                 splitImage(img_PIL3, cpt)
            cpt = 1
            #dessin de contours
            contours, hierarchy = cv2.findContours(img_PIL4, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            #on peut faire afficher le contour qu'il trouve pour voir s'il marche bien
#            out = np.zeros_like(seg_sat)
#            cv2.drawContours(out, contours, -1, 255, 3)
#            cv2.imshow('drawContours', out)
#            cv2.waitKey(0)
#            cv2.destroyAllWindows()

            # Algorithme de calcul de la meilleure aire representative d'obstacle
            best_area = 0
            omega = 0.0
            vitesse = 0.0
            #on parcours les contours trouves et on choisit le plus grand
            for c in contours:
                area = cv2.contourArea(c)
                if area > best_area: best_area, best_c = area, c
                
            print "aire trouvee : ", best_area    
            
            #si l'aire trouvee est plus grande que 0
            if best_area > 0:
                # on calcule le barycentre de cette aire
                M = cv2.moments(best_c)
                cx, cy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
                #transformation de "pixel" a angles
                cx = cx * Ang_x/imageWidth
                cy = cy * Ang_y/imageHeight
                #calcul de la difference d'angles entre le barycentre et le robot
                dx = cx - midAng_x
                dy = cy - midAng_y
    
                # on recupere les donnees de position de la tete
                x, y = recordData()
                #on ajoute les differentiels
                x, y = x* Ang_x/imageWidth + dx, y*Ang_y/imageHeight + dy
                omega = -x/2
                print " | dx : ", dx, " | dy : ", dy
                #on effectue les mouvements de la tete
                motionProxy.setAngles("HeadYaw", -x*almath.TO_RAD, 0.5)
                motionProxy.setAngles("HeadPitch", y*almath.TO_RAD, 0.5)

            #theta egal a l'angle de rotation necessaire de la tete
            #on suppose qu'on voudrait y arriver en 1 seconde
            #donc la norme de theta egale la norme de x
            if omega < -0.7 : omega = -0.7
            if omega > +0.7 : omega = +0.7
            #on ajoute un maximum a omega pour garder l'equilibre
            #le maximum correspond au decalage maximum en une seconde (il ne peut pas aller plus loin en agnle que ce qu'il voit)
            if best_area > seuilHaut:
                vitesse = -bangbang
                setLeds(255, 0, 0)
                #led rouge si l'obstacle ets trop pres
            elif best_area < seuilBas and best_area > 2:
                vitesse = +bangbang
                setLeds(0, 0, 255)
                #led bleu si l'obstacle est tres eloigne
            elif best_area > seuilSecu:
                vitesse = 0.0
                setLeds(200,200,200)
                #led blanche si on se place en securite
            elif best_area == 0:
                vitesse = 0
                omega = 0
                setLeds(255,255,0)
                #led jaune si on ne detecte rien
            else:
                setLeds(0, 255, 0)
                vitesse = 0.0
                #led verte si l'obstacle est a la bonne distance
            if omega > -0.2 or omega < +0.2: omega = 0.0
            
            #on applique les vitesses     
            print 'vitesse et theta', vitesse, omega    
            motionProxy.post.setWalkTargetVelocity(vitesse, 0.0, omega, 0.3)
            
            #utile si l'on veut faire un essai sur un tour de boucle
#            filein=open("cmd","w")
#            filein.write("stop")
#            filein.close() 
            
        elif cmd=="stop":
            print "je m'arrete"
            motionProxy.stopMove()
            Cancel()
           
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
        
        
