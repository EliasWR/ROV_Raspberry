import socket
import pickle
import cv2
import numpy as np
import config
import threading
import struct
import math
from imutils.video import VideoStream
import serial
import json
import time

class FrameSegment(object):
  """“””
  Object to break down image frame segment
  if the size of image exceed maximum datagram size
  “””"""

  def __init__(self, sock, port, addr="169.254.226.73"):
    self.s = sock
    self.port = port
    self.addr = addr
    self.MAX_DGRAM = 2**16
    self.MAX_IMAGE_DGRAM = self.MAX_DGRAM - 64 # minus 64 bytes in case UDP frame overflown

  def udp_frame(self, img):
    """“””
    Compress image and Break down
    into data segments
    “””"""
    # compress_img = cv2.resize(img, None, fx = 0.4, fy=0.4, interpolation=cv2.INTER_AREA) # 2 420 000 bytes
    compress_img = cv2.resize(img, None, fx = 0.1, fy=0.1, interpolation=cv2.INTER_AREA)    # 150 802 bytes
    compress_img = cv2.imencode(".jpg", compress_img)[1]

    dat = compress_img.tostring()
    size = len(dat)
    num_of_segments = math.ceil(size/(self.MAX_IMAGE_DGRAM))
    array_pos_start = 0

    while num_of_segments:
      array_pos_end = min(size, array_pos_start + self.MAX_IMAGE_DGRAM)
      self.s.sendto(
                   struct.pack("B", num_of_segments) +
                   dat[array_pos_start:array_pos_end],
                   (self.addr, self.port)
                   )
      array_pos_start = array_pos_end
      num_of_segments -= 1





def TCPIn():

    HEADERSIZE = 10

    while 1: # Constantly checking for new messages
        receiving = True
        full_msg = b''
        new_msg = True
        incoming_message = config.clientsocket.recv(8192)

        while receiving:

            if new_msg:    # Changed rom if new_msg
                msglen = int(incoming_message[:HEADERSIZE])
                new_msg = False

            full_msg += incoming_message

            if len(full_msg)-HEADERSIZE == msglen:
                GuiDataIn = pickle.loads(full_msg[HEADERSIZE:])  # Deserialize reponse from GUI
                print("[ATTENTION] New data has been applied to global variables from GUI commands")
                # Setting the global variables accordingly
                config.light = GuiDataIn["light"]
                config.runZone = GuiDataIn["runZone"]
                config.forceReset = GuiDataIn["forceReset"]
                config.mode = GuiDataIn["mode"]
                # changeOperatingMode(GuiDataIn["mode"])  # SET BY GUI
                config.newArduinoCommands = True

                # Resetting variables for next iteration
                receiving = False
                new_msg = True
                full_msg = b''


def TCPOut(s, HOST, PORT, HEADERSIZE):
    communicating = True
    startReceive = True

    while communicating:
        receiving = True    # When beginning while loop we want to receive a message from GUI

        if not config.address: # If no address is given, try to find one
            config.clientsocket, config.address = s.accept()
            print(f"Connection from {config.address} has been established.")

        """
        TODO: Implement logic that can take a high definition picture and send it over TCPIn
        """

        # ret_val, img = cam.read()   # Takes a photo with camera
        # compressed_img = commpressImage(img, 5)    # Compressing image
        # frame = vs.read()
        # resized = imutils.resize(frame, width=600)

        # Finalizing dicitionary with all values to be sent to GUI
        GuiDataOut = {
            "image": "",
            "temp": config.temp,
            "pressure": config.pressure,
            "leak": config.leak,
            "angle": config.angle,
            "step": config.step,
            "lockedZones": config.interlockedZones,
            "dataArray": config.data_lst
        }

        # Serializing the dicitionary and sending
        msg = pickle.dumps(GuiDataOut)
        msg = bytes(f"{len(msg):<{HEADERSIZE}}", 'utf-8') + msg
        config.clientsocket.send(msg)

        communicating = False

def UDP():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    port = 20001
    fs = FrameSegment(s, port)

    # cap = cv2.VideoCapture(0)
    vs = VideoStream(src=0).start()

    while 1:
        # _, frame = cap.read()

        frame = vs.read()
        fs.udp_frame(frame)

    cap.release()
    cv2.destroyAllWindows()
    s.close()

def serialCom():

    # Initialize serial communication with Arduino UNO
    ardSer = serial.Serial('/dev/ttyACM0', 9600, timeout=1,
    parity=serial.PARITY_NONE, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE)
    print(f'Arduino serial communication status: {ardSer.isOpen()}')


    condSer = serial.Serial('/dev/ttyUSB0', 9600)
    print(f'Conductivity sensor communication status: {condSer.isOpen()}')

    time.sleep(2)

    while 1:
        """
        SERIAL WITH CONDUCTIVITY SENSOR
        """
        condSer.write("do_sample\n".encode())   # Commands conductivity sensor to conduct sample of parameters

        salinityReading = getAanderaaData(condSer, "get_salinity\n")
        soundSpeedReading = getAanderaaData(condSer, "get_soundspeed\n")
        densityReading = getAanderaaData(condSer, "get_density\n")
        conductivityReading = getAanderaaData(condSer, "get_conductivity\n")

        print(f'Salinity = {salinityReading}')
        print(f'Sound of speed: {soundSpeedReading}')
        print(f'Conductivity = {conductivityReading}')
        print(f'density: {densityReading}')


        """
        SERIAL WITH ARDUINO
        """

        if config.newArduinoCommands:
            ArdDataOut = {}
            ArdDataOut["light"] = config.light # SET BY GUI: 0-255 light settings
            ArdDataOut["runZone"] = config.runZone # SET BY GUI: 1-8 Linear directions, 9 and 10 clock and counter-clock respectively
            ArdDataOut["locked"] = config.interlockedZones
            ArdDataOut = json.dumps(ArdDataOut)
            ardSer.write(ArdDataOut.encode())
            config.newArduinoCommands = False

        # If program takes longer to run, there might be problem with serial
        # Band-aid solution could be to add delay in Arduino C++ script
        if ardSer.in_waiting > 0:
            ArdDataIn = json.loads(ardSer.readline())   # Deserializes input from Arduino
            config.temp = ArdDataIn["Temp"]
            config.pressure = ArdDataIn["Pressure"]
            config.leak = ArdDataIn["Leak"]
            # ardSer.write(ArdDataOut.encode())   # Responds with data to Arduino

        condIn = condSer.readline()  # Have to read the buffer to stop future splitting issues





"""
Can be used with the picture taking functionality used in TCPOut communication
"""
def commpressImage(img, k):
    width = int((img.shape[1])/k)
    height = int((img.shape[0])/k)
    return cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)

def getAanderaaData(condSer, request_str):
    condIn = b''
    condIn = condSer.readline()  # Have to read the buffer to stop future splitting issues
    condSer.write(request_str.encode())
    condIn = condSer.readline().decode()
    data = condIn.split('\t')
    data = data[-1]
    data = data.replace('\r\n', '')
    return float(data)