#!/usr/bin/env python

# # Python libs
# import sys, time

# # numpy and scipy
# import numpy as np
# #from scipy.ndimage import filters

# import imutils

# # OpenCV
# import cv2

# Ros libraries
# import roslib
import rospy


# Ros Messages
from control_arduino.msg import RobotCommand

import serial
import threading
import time
velocity = None
startMarker = '<'
endMarker = '>'
dataStarted = False
dataBuf = ""
messageComplete = False
go=True


# ========================
# ========================
# the functions


def rec():
	arduinoReply = recvLikeArduino()
	if not (arduinoReply == 'XXX'):
		print ("Time %s  Reply %s" % (time.time(), arduinoReply))


def setupSerial(baudRate, serialPortName):

	global  serialPort
	serialPort = serial.Serial(port= serialPortName, baudrate = baudRate, timeout=0, rtscts=True)
	print("Serial port " + serialPortName + " opened  Baudrate " + str(baudRate))
	waitForArduino()


# ========================

def sendToArduino(stringToSend):

	# this adds the start- and end-markers before sending
	global startMarker, endMarker, serialPort

	stringWithMarkers = (startMarker)
	stringWithMarkers += stringToSend
	stringWithMarkers += (endMarker)
	serialPort.write(stringWithMarkers.encode('utf-8')) # encode needed for Python3
	print(stringToSend)


# ==================

def recvLikeArduino():

	global startMarker, endMarker, serialPort, dataStarted, dataBuf, messageComplete

	# print("here!")

	if serialPort.inWaiting() > 0 and messageComplete == False:
		x = serialPort.read().decode("utf-8") # decode needed for Python3

		if dataStarted == True:
			if x != endMarker:
				dataBuf = dataBuf + x
			else:
				dataStarted = False
				messageComplete = True
		elif x == startMarker:
			dataBuf = ''
			dataStarted = True
	# print("here! 2")
	if (messageComplete == True):
		messageComplete = False
		return dataBuf
	else:
		return "XXX"

	# ==================

def waitForArduino():

	# wait until the Arduino sends 'Arduino is ready' - allows time for Arduino reset
	# it also ensures that any bytes left over from a previous message are discarded

	print("Waiting for Arduino to reset")

	msg = ""
	while msg.find("Arduino is ready") == -1:
		msg = recvLikeArduino()
		if not (msg == 'XXX'):
			print(msg)


# ====================nano
# ====================
# the program


setupSerial(115200, "/dev/ttyACM0")
# setupSerial(115200, "/dev/ttyACM0")
# count = 0
# prevTime = time.time()
# vel = 100
# inc = 10
# switch = 0




def callback(data):
	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
	#g = raw_input(data)     #serve per dare l'input ad arduino
	# print("here")
	global velocity
	newData = str(data)
	# print("velocity")
	# print(velocity)
	# print("print(newData)")
	print(newData)
	if velocity != newData:
		velocity = newData
		sendToArduino(newData)
	#if(data=="stop"):
	#   sendToArduino("+0.0")
	#   time.sleep(1)
	#   return
	#for n in range(0,1000000):
	#rec()




def main():
	#'''Initializes and cleanup ros node'''
	rospy.init_node('sender', anonymous=True)
	rospy.Subscriber("/robot_command", RobotCommand, callback,  queue_size = 10)
	rospy.spin()


if __name__ == '__main__':
	main()
