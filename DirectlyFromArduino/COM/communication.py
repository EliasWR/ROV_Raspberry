"""
RASPBERRY PI SUB PROGRAM CONTAINING THE LOGIC TO HANDLE 
THE COMMUNICATION BETWEEN ALL DEVICES CONNECTED TO THE RPI.
INCLUDES SERIAL COMMUNICATION, TCP AND UDP.
"""

import socket
import pickle
import numpy as np
import config
import threading
import struct
import math
from imutils.video import VideoStream
import serial
import json
import time
import cv2


class FrameSegment(object):
 
    # Initialization of functionality that handles dividing picture frames to correctly sized UDP datagrams
    def __init__(self, sock, port, addr="169.254.226.73"):
        self.s = sock
        self.port = port
        self.addr = addr
        self.MAX_DGRAM = 2**16
        self.MAX_IMAGE_DGRAM = self.MAX_DGRAM - 64

    # Function that takes in a frame, compresses it, divides it into UDP datagrams and 
    # sends it over UDP to the GUI
    def udp_frame(self, img):
        # Compress image to .jpg format
        compress_img = cv2.imencode(".jpg", img)[1]
        dat = compress_img.tostring()
        size = len(dat)
        # Finds number of datagrams needed to be sent for this frame
        num_of_segments = math.ceil(size/(self.MAX_IMAGE_DGRAM))
        array_pos_start = 0
        
        # Sends out all the datagrams needed for the frame
        while num_of_segments:
            array_pos_end = min(size, array_pos_start + self.MAX_IMAGE_DGRAM)
            self.s.sendto(
                struct.pack("B", num_of_segments) +
                dat[array_pos_start:array_pos_end],
                (self.addr, self.port)
                )
            array_pos_start = array_pos_end
            num_of_segments -= 1




# Function that handles the TCP data coming from the GUI
def TCPIn():

    HEADERSIZE = 10

    # Constantly checking for new messages
    while 1: 
        receiving = True
        full_msg = b''
        new_msg = True
        incoming_message = config.clientsocket.recv(8192)

        while receiving:

            if new_msg:  
                msglen = int(incoming_message[:HEADERSIZE])
                new_msg = False

            full_msg += incoming_message

            # If full message received, update global variables
            if len(full_msg)-HEADERSIZE == msglen:
                GuiDataIn = pickle.loads(full_msg[HEADERSIZE:]) 
                print("[ATTENTION] New data has been applied to global variables from GUI commands")
                config.light = GuiDataIn["light"]
                config.motorSpeed = GuiDataIn["motorSpeed"]
                config.runZone = GuiDataIn["runZone"]
                config.forceReset = GuiDataIn["forceReset"]
                config.mode = GuiDataIn["mode"]
                config.takeHighResPhoto = GuiDataIn["takePhoto"]
                config.takeVideo = GuiDataIn["takeVideo"]
                config.newArduinoCommands = True

                # Resetting variables for next iteration
                receiving = False
                new_msg = True
                full_msg = b''


# Function that handles the TCP data to be sent to the GUI
def TCPOut(s, HOST, PORT, HEADERSIZE):
    communicating = True
    startReceive = True

    while communicating:
        receiving = True   

        # If no connection is established, try to find one
        if not config.address: 
            config.clientsocket, config.address = s.accept()
            print(f"Connection from {config.address} has been established.")

        # Finalizing dicitionary with all values to be sent to GUI
        GuiDataOut = {
            "image": "",
            "temp": config.temp,
            "depth": config.depth,
            "leak": config.leak,
            "angle": config.angle,
            "step": config.step,
            "lockedZones": config.interlockedZones,
            "dataArray": config.data_lst,
            "salinity": config.salinity,
            "conductivity": config.conductivity,
            "density": config.density

        }

        # Serializing the dicitionary and sending
        msg = pickle.dumps(GuiDataOut)
        msg = bytes(f"{len(msg):<{HEADERSIZE}}", 'utf-8') + msg
        config.clientsocket.send(msg)

        communicating = False


# Function that handles the UDP communication with GUI
def UDP():
    # Establish connection with server
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    port = 20001

    # Declare object for handling image frames 
    fs = FrameSegment(s, port)

    # Variables for naming photo and video files
    photoNum = 0
    videoNum = 0

    # Opens camera port and defines video format
    cap = VideoStream(src=0).start()
    size = (640, 480)

    while 1:
        # If commanded from GUI, take photo and save to determined path
        if config.takeHighResPhoto:

            photoNum += 1
            photo = cap.read()

            status = cv2.imwrite(f'/home/pi/Programs/Photos/photo_{photoNum}.png', photo)
            print(f'Image written to file system status: {status}')
            time.sleep(1)   # Sleeps for 1 second before resuming UDP video stream
            config.takeHighResPhoto = False

        # If no commands to take picture, resume video stream over UDP to GUI
        while not config.takeHighResPhoto:
            frame = cap.read()

            # If user commands to save video, store video to file
            if config.takeVideo:
                if not vidConfigured:
                    videoNum += 1
                    result = cv2.VideoWriter(f'/home/pi/Programs/Videos/Video{videoNum}.avi',
                    cv2.VideoWriter_fourcc(*'mp4v'), 12, size)
                    vidConfigured = True

                result.write(frame) # Writing to disk as a video
            else:
                vidConfigured = False

            fs.udp_frame(frame) # Sending to GUI using UDP communication

    cap.release()
    cv2.destroyAllWindows()
    s.close()


# Function that handles serial communication with Arduino Uno and combination sensor
def serialCom():

    # Initialize serial communication with Arduino UNO
    ardSer = serial.Serial('/dev/ttyACM0', 9600, timeout=1,
    parity=serial.PARITY_NONE, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE)
    print(f'Arduino serial communication status: {ardSer.isOpen()}')

    # Initialize serial communication with conductivity/combination sensor
    condSer = serial.Serial('/dev/ttyUSB1', 9600)
    print(f'Conductivity sensor communication status: {condSer.isOpen()}')

    time.sleep(2)

    # Continuously send and receive over serial connection
    while 1:
       
        # Commands conductivity sensor to conduct sample of values
        condSer.write("do_sample\n".encode())   

        # Commands for values, and reads response to global variables
        config.salinity = getAanderaaData(condSer, "get_salinity\n")
        soundSpeedReading = getAanderaaData(condSer, "get_soundspeed\n")
        config.density = getAanderaaData(condSer, "get_density\n")
        config.conductivity = getAanderaaData(condSer, "get_conductivity\n")

        # If new commands has been updated for Arduino Uno, structure data from global variable, serialize and send
        if config.newArduinoCommands:
            ArdDataOut = {}
            ArdDataOut["light"] = config.light 
            ArdDataOut["runZone"] = config.runZone 
            ArdDataOut["locked"] = config.interlockedZones
            ArdDataOut = json.dumps(ArdDataOut)
            ardSer.write(ArdDataOut.encode())
            config.newArduinoCommands = False

        # If the serial input buffer reserved for Arduino traffic has data, unserialize and store in global variables
        if ardSer.in_waiting > 0:
            ArdDataIn = json.loads(ardSer.readline())   # Deserializes input from Arduino
            config.temp = ArdDataIn["Temp"]
            config.depth = ArdDataIn["Depth"]
            config.leak = ArdDataIn["Leak"]

        # Have to read and purge input buffer for combination sensor, as old data can ruin future readings
        condIn = condSer.readline()  


# Function that compresses frame read from camera
def commpressImage(img, k):
    width = int((img.shape[1])/k)
    height = int((img.shape[0])/k)
    return cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)

# Function that sends commands and reads response. Reads value 
# based on given parameter
def getAanderaaData(condSer, request_str):
    condIn = b''
    condIn = condSer.readline()  # Have to read the buffer to stop future splitting issues
    condSer.write(request_str.encode())
    condIn = condSer.readline().decode()
    data = condIn.split('\t')
    data = data[-1]
    data = data.replace('\r\n', '')
    return float(data)