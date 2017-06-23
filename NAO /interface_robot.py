
from PyQt4 import QtGui

import sys
import os

app = QApplication(sys.argv)
    
    
QtGui.QWidget.b_main_1=QtGui.QLabel(QtGui.QWidget)
QtGui.QWidget.b_main_1.setText('Bienvenue')
QtGui.QWidget.b_main_1.move(608.5,235)

QtGui.QWidget.b_main_2 = QtGui.QLabel(QtGui.QWidget)
QtGui.QWidget.b_main_2.setText('vous pouvez tester votre robot en utilisant les boutons ci-dessous ')
QtGui.QWidget.b_main_2.move(540,270)

QtGui.QWidget.b_main_3 = QtGui.QPushButton(QtGui.QWidget)
QtGui.QWidget.b_main_3.setText("test marche ")
QtGui.QWidget.b_main_3.clicked.connect(test_marche)
QtGui.QWidget.b_main_3.move(598.5,325)

QtGui.QWidget.b_main_4 = QtGui.QPushButton(QtGui.QWidget)
QtGui.QWidget.b_main_4.setText("test articulation")
QtGui.QWidget.b_main_4.clicked.connect(test_articulation)
QtGui.QWidget.b_main_4.move(598.5,355)

QtGui.QWidget.b_main_5 = QtGui.QPushButton(QtGui.QWidget)
QtGui.QWidget.b_main_5.setText("test parole")
QtGui.QWidget.b_main_5.clicked.connect(test_parole)
QtGui.QWidget.b_main_5.move(598.5,375)

""" creation de widget 1"""

QtGui.QWidget.w_1 = QtGui.QWidget(QtGui.QWidget)
QtGui.QWidget.w_1.setGeometry(5,32,1270,730)
QtGui.QWidget.w_1.hide()

""" creation de widget 2"""

QtGui.QWidget.w_2 = QtGui.QWidget(QtGui.QWidget)
QtGui.QWidget.w_2.setGeometry(5,32,1270,730)
QtGui.QWidget.w_2.hide()

""" creation de widget 3"""

QtGui.QWidget.w_3 = QtGui.QWidget(QtGui.QWidget)
QtGui.QWidget.w_3.setGeometry(5,32,1270,730)
QtGui.QWidget.w_3.hide()

"""definition des texte et creation des bouttons pour widget 1"""

QtGui.QWidget.b_1_1 = QtGui.QLabel(QtGui.QWidget.w_1)
QtGui.QWidget.b_1_1.setText("testez la mouvement de votre robot")
QtGui.QWidget.b_1_1.move(579,290)

QtGui.QWidget.b_1_2 = QtGui.QPushButton('marche avant ', QtGui.QWidget.w_1)
QtGui.QWidget.b_1_2.move(800, 310)
QtGui.QWidget.b_1_2.clicked.connect()

QtGui.QWidget.b_1_3 = QtGui.QPushButton('marche arriere ', QtGui.QWidget.w_1)
QtGui.QWidget.b_1_3.move(800, 330)
QtGui.QWidget.b_1_3.clicked.connect()

QtGui.QWidget.b_1_4 = QtGui.QPushButton('tourne a gauche ', QtGui.QWidget.w_1)
QtGui.QWidget.b_1_4.move(800, 350)
QtGui.QWidget.b_1_4.clicked.connect()

QtGui.QWidget.b_1_5 = QtGui.QPushButton('tourne a droite', QtGui.QWidget.w_1)
QtGui.QWidget.b_1_5.move(800, 370)
QtGui.QWidget.b_1_5.clicked.connect()

QtGui.QWidget.b_1_6= QtGui.QPushButton(QtGui.QWidget.w_1)
QtGui.QWidget.b_1_6.setText("retour")
QtGui.QWidget.b_1_6.clicked.connect(QtGui.QWidget.page)
QtGui.QWidget.b_1_6.move(42.5,20)

"""definition des texte et creation des bouttons pour widget 2"""

QtGui.QWidget.b_2_1 = QtGui.QLabel(QtGui.QWidget.w_2)
QtGui.QWidget.b_2_1.setText("testez les articulations de votre robot ")
QtGui.QWidget.b_2_1.move(579,290)

QtGui.QWidget.b_2_2 = QtGui.QPushButton('marche avant ', QtGui.QWidget.w_2)
QtGui.QWidget.b_2_2.move(800, 310)
QtGui.QWidget.b_2_2.clicked.connect()

QtGui.QWidget.b_2_3 = QtGui.QPushButton('marche arriere ', QtGui.QWidget.w_2)
QtGui.QWidget.b_2_3.move(800, 330)
QtGui.QWidget.b_2_3.clicked.connect()

QtGui.QWidget.b_2_4 = QtGui.QPushButton('tourne a gauche ', QtGui.QWidget.w_2)
QtGui.QWidget.b_2_4.move(800, 350)
QtGui.QWidget.b_2_4.clicked.connect()

QtGui.QWidget.b_2_5 = QtGui.QPushButton('tourne a droite', QtGui.QWidget.w_2)
QtGui.QWidget.b_2_5.move(800, 370)
QtGui.QWidget.b_2_5.clicked.connect()

QtGui.QWidget.b_2_6= QtGui.QPushButton(QtGui.QWidget.w_2)
QtGui.QWidget.b_2_6.setText("retour")
QtGui.QWidget.b_2_6.clicked.connect(QtGui.QWidget.page)
QtGui.QWidget.b_2_6.move(42.5,20)

"""definition des texte et creation des bouttons pour widget 3"""

QtGui.QWidget.b_3_1 = QtGui.QLabel(QtGui.QWidget.w_3)
QtGui.QWidget.b_3_1.setText("testez la mouvement de votre robot")
QtGui.QWidget.b_3_1.move(579,290)

QtGui.QWidget.b_3_2 = QtGui.QPushButton('marche avant ', QtGui.QWidget.w_3)
QtGui.QWidget.b_3_2.move(800, 310)
QtGui.QWidget.b_3_2.clicked.connect()

QtGui.QWidget.b_3_3 = QtGui.QPushButton('marche arriere ', QtGui.QWidget.w_3)
QtGui.QWidget.b_3_3.move(800, 330)
QtGui.QWidget.b_3_3.clicked.connect()

QtGui.QWidget.b_3_4 = QtGui.QPushButton('tourne a gauche ', QtGui.QWidget.w_3)
QtGui.QWidget.b_3_4.move(800, 350)
QtGui.QWidget.b_3_4.clicked.connect()

QtGui.QWidget.b_3_5 = QtGui.QPushButton('tourne a droite', QtGui.QWidget.w_3)
QtGui.QWidget.b_3_5.move(800, 370)
QtGui.QWidget.b_3_5.clicked.connect()

QtGui.QWidget.b_3_6= QtGui.QPushButton(QtGui.QWidget.w_1)
QtGui.QWidget.b_3_6.setText("retour")
QtGui.QWidget.b_3_6.clicked.connect(QtGui.QWidget.page)
QtGui.QWidget.b_3_6.move(42.5,20)
    
      
def page(Widget):

    Widget.w_1.hide()
    Widget.w_2.hide()
    Widget.w_3.hide()
    
    Widget.b_main_1.show()
    Widget.b_main_2.show()
    Widget.b_main_3.show()
    Widget.b_main_4.show()
    Widget.b_main_5.show()

def test_marche(Widget):

    Widget.b_main_1.hide()
    Widget.b_main_2.hide()
    Widget.b_main_3.hide()
    Widget.b_main_4.hide()
    Widget.w_1.show()

    Widget.w_2.hide()
    Widget.w_3.hide()

def test_articulation(Widget):

    Widget.b_main_1.hide()
    Widget.b_main_2.hide()
    Widget.b_main_3.hide()
    Widget.b_main_4.hide()
    Widget.w_2.show()

    Widget.w_1.hide()
    Widget.w_3.hide()

def test_parole(Widget):

    Widget.b_main_1.hide()
    Widget.b_main_2.hide()
    QtGui.QWidget.b_main_3.hide()
    QtGui.QWidget.b_main_4.hide()
    QtGui.QWidget.w_3.show()

    QtGui.QWidget.w_1.hide()
    QtGui.QWidget.w_2.hide()

    
   
    
    
myWidget = vis.ImageWidget(robotIP, port, CameraID)
myWidget.show()
boutton= QPushButton()
boutton.show()
boutton.clicked.connect(close)

QtGui.QWidget.setGeometry(5,32,1270,730)
QtGui.QWidget.show()

sys.exit(app.exec_())