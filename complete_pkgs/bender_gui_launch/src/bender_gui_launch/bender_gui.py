#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import ros
import roslib
from PyQt4 import QtCore, QtGui
import time

import sys
import os
import subprocess

import rc_launch

class Ui_MainWindow(rc_launch.Ui_MainWindow):

    def __init__(self):
        self.currentTest = {'launch':'','test':''}
        self.chestOK = False
        self.greenOK = False
        self.blueOK = False
        self.roscore = self.start_roscore()
#        while self.roscore.poll() != None:
#            continue

    def start_roscore(self):
        roscore = subprocess.Popen(['roscore'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        rospy.sleep(1)
        return roscore

    def launchChest(self):
        #print 'Boton 1 apretado'
        launch_exec = 'roslaunch bender_behaviors '+self.currentTest['launch']+'_gray.launch &'
        self.launch_msg.append(launch_exec)
        #self.launch_proc = subprocess.Popen([launch_exec], stdout=subprocess.PIPE, shell=True)  
        try:
            os.system('terminator -x '+str(launch_exec))
            self.chestOK = True
        except:
            pass

    def startTest(self):
        #print 'Boton 2 apretado'
        launch_test = 'rosrun bender_behaviors '+self.currentTest['test']+'.py $'

        if self.chestOK is True:
            self.launch_msg.append(launch_test)
            os.system('terminator -x '+str(launch_test))
        else:
            self.launch_msg.append('Launch Chest First!')

        #self.test_proc = subprocess.Popen([launch_test], stdout=subprocess.PIPE, shell=True)

        #while self.test_proc.poll() is None:
            #l = self.test_proc.stdout.readline() # This blocks until it receives a newline.
            #print l
        # When the subprocess terminates there might be unconsumed output 
        # that still needs to be processed.
        #print self.test_proc.stdout.read()

    def updateTest(self):
        self.launch_msg.append('Test '+self.comboBox.currentText()+' Loaded')

        if self.comboBox.currentText() == "Navigation Test":
            self.currentTest['launch'] = 'roslaunch bender_behaviors complete_nav.launch'
            self.currentTest['test'] = 'rosrun bender_behaviors Navigation.py'
        elif self.comboBox.currentText() == "Speech Recognition and Audio Detection Test":
            self.currentTest['launch'] = 'SpeechRecognition'
            self.currentTest['test'] = 'SpeechRecognition'
            self.blueOK = True
            self.greenOK = True
        elif self.comboBox.currentText() == "RoboZoo":
            self.currentTest['launch'] = 'roslaunch bender_speech complete_speech.launch'
            self.currentTest['test'] = 'rosrun bender_macros AskQuestion.py'
        elif self.comboBox.currentText() == "General Purpose Service Robot":
            self.currentTest['launch'] = 'GPSR'
            self.currentTest['test'] = 'GPSR'
        elif self.comboBox.currentText() == "Manipulation Test":
            self.currentTest['launch'] = 'Manipulation'
            self.currentTest['test'] = 'Manipulation'
        elif self.comboBox.currentText() == "Navigation Test":
            self.currentTest['launch'] = 'roslaunch bender_speech complete_speech.launch'
            self.currentTest['test'] = 'rosrun bender_macros AskQuestion.py'
        elif self.comboBox.currentText() == "Person Recognition Test":
            self.currentTest['launch'] = 'roslaunch bender_speech complete_speech.launch'
            self.currentTest['test'] = 'rosrun bender_macros AskQuestion.py'

    def launchBlue(self):

        launch_blue = 'roslaunch bender_behaviors '+self.currentTest['launch']+'_blue.launch &'
        #self.launch_msg.append(launch_exec)
        #self.launch_proc = subprocess.Popen([launch_exec], stdout=subprocess.PIPE, shell=True)  
        try:
            os.system('terminator -x '+str(launch_blue))
            self.blueOK = True
        except:
            pass

    def launchGreen(self):

        launch_green = 'roslaunch bender_behaviors '+self.currentTest['launch']+'_green.launch &'
        #self.launch_msg.append(launch_exec)
        #self.launch_proc = subprocess.Popen([launch_exec], stdout=subprocess.PIPE, shell=True)  
        try:
            os.system('terminator -x '+str(launch_green))
            self.blueOK = True
        except:
            pass

    def destructor(self):
        self.roscore.terminate()


class RobocupGUI(QtGui.QMainWindow):

    def __init__(self):
        super(RobocupGUI,self).__init__()

        self.initUI()

    def initUI(self):

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.show()

    def closeEvent(self,event):
        result = QtGui.QMessageBox.question(self,
                      "Confirm Exit...",
                      "Are you sure you want to exit ?\nroscore will be killed",
                      QtGui.QMessageBox.Yes| QtGui.QMessageBox.No)
        event.ignore()

        #self.ui.launch_msg.append( str(self.ui.roscore.pid ) )
        #print str(self.ui.roscore.pid )

        if result == QtGui.QMessageBox.Yes:
            self.ui.roscore.terminate()
            rospy.sleep(2)
            event.accept()



if __name__ == "__main__":
    #rospy.init_node('robot_gui')

    app = QtGui.QApplication(sys.argv)
    ui = RobocupGUI()
    sys.exit(app.exec_())

