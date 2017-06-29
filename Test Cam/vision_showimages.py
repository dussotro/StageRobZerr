# -*- encoding: UTF-8 -*-
#
# This is a tiny example that shows how to show live images from Nao using PyQt.
# You must have python-qt4 installed on your system.
#

import sys
import cv2
from PIL import Image
from PyQt4.QtGui import QWidget, QImage, QApplication, QPainter
from naoqi import ALProxy
import numpy

# To get the constants relative to the video.
import vision_definitions


class ImageWidget(QWidget):
    """
    Tiny widget to display camera images from Naoqi.
    """
    def __init__(self, IP, PORT, CameraID, parent=None):
        """
        Initialization.
        """
        QWidget.__init__(self, parent)
        self._image = QImage()
        self.setWindowTitle('Nao')

        self._imgWidth = 320
        
        self._imgHeight = 240
        self._cameraID = CameraID
        self.resize(self._imgWidth, self._imgHeight)

        # Proxy to ALVideoDevice.
        self._videoProxy = None

        # Our video module name.
        self._imgClient = ""

        # This will contain this alImage we get from Nao.
        self._alImage = None

        self._registerImageClient(IP, PORT)

        # Trigget 'timerEvent' every 100 ms.
        self.startTimer(100)


    def _registerImageClient(self, IP, PORT):
        """
        Register our video module to the robot.
        """
        self._videoProxy = ALProxy("ALVideoDevice", IP, PORT)
        resolution = vision_definitions.kQVGA  # 320 * 240
        colorSpace = vision_definitions.kRGBColorSpace
        try :
            self._imgClient = self._videoProxy.subscribe("_client", resolution, colorSpace, 5)
        except :
            self._videoProxy.unsuscribe("_client")
            self._imgClient = self._videoProxy.subscribe("_client", resolution, colorSpace, 5)
        # Select camera.
        self._videoProxy.setParam(vision_definitions.kCameraSelectID,
                                  self._cameraID)


    def _unregisterImageClient(self):
        """
        Unregister our naoqi video module.
        """
        if self._imgClient != "":
            self._videoProxy.unsubscribe(self._imgClient)


    def paintEvent(self, event):
        """
        Draw the QImage on screen.
        """
        painter = QPainter(self)
        painter.drawImage(painter.viewport(), self._image)


    def _updateImage(self):
        """
        Retrieve a new image from Nao.
        """
        from_video = self._videoProxy.getImageRemote(self._imgClient)
        img_nao = from_video[6]
        img_PIL = Image.frombytes("RGB", (self._imgWidth, self._imgHeight), img_nao)
        img_PIL = cv2.cvtColor(numpy.asarray(img_PIL), cv2.COLOR_RGB2HSV)
        img_PIL = cv2.blur(img_PIL, (5,5))
            
        hue = cv2.split(img_PIL)[0]
        sat = cv2.split(img_PIL)[1]
        val = cv2.split(img_PIL)[2]
        
        seuil = 100.0
        ret, seg_sat = cv2.threshold(sat, seuil, 255.0, cv2.THRESH_BINARY)
        
        contours, hierarchy = cv2.findContours(seg_sat, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = numpy.zeros_like(seg_sat)
        cv2.drawContours(out, contours, -1, 255, 3)   
        self._pixelArray = out
        
#        self._alImage = self._videoProxy.getImageRemote(self._imgClient)
        self._image = QImage(self._pixelArray,         # Pixel array.
                             self._imgWidth,           # Width.
                             self._imgHeight,          # Height.
                             QImage.Format_RGB888)


    def timerEvent(self, event):
        """
        Called periodically. Retrieve a nao image, and update the widget.
        """
        self._updateImage()
        self.update()


    def __del__(self):
        """
        When the widget is deleted, we unregister our naoqi video module.
        """
        self._unregisterImageClient()



if __name__ == '__main__':
    IP = "172.20.28.103"  # Replace here with your NaoQi's IP address.
    PORT = 9559
    CameraID = 0 # 0: top | 1: bottom

    # Read IP address from first argument if any.
    if len(sys.argv) > 1:
        IP = sys.argv[1]

    # Read CameraID from second argument if any.
    if len(sys.argv) > 2:
        CameraID = int(sys.argv[2])


    app = QApplication(sys.argv)
    myWidget = ImageWidget(IP, PORT, CameraID)
    myWidget.show()
    sys.exit(app.exec_())
