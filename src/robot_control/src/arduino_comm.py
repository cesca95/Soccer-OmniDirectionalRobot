
import serial
import time
import threading

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

# BL/BR/FL/FR

setupSerial(115200, "/dev/ttyACM0")
count = 0
prevTime = time.time()
vel = 100
inc = 10
switch = 0
print("Enter the velocity (rps) with VVVS VVVS VVVS VVVS, as 19 characters VVV for velocity between 000 to 255, and Switch(S) as 0 or 1 ")
while True:
    g = raw_input("'VVVS VVVS VVVS VVVS' in format BL/BR/FL/FR, type stop to exit the code: ")
    sendToArduino(g)
    if(g=="stop"):
        sendToArduino("+0.0")
        time.sleep(1)
        break
    for n in range(0,1000000):
        rec()