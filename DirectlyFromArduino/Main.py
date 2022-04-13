#!/usr/bin/env python3
import serial
import time
import numpy as np
import cv2
import json
import threading
from math import *
import socket
import pickle    # Serialization of data for GUI communication
import argparse

# Custom libraries imports
from sonarFunctionality.BlueRoboticsSonar import Ping360
from sonarFunctionality.Interlocking import InterlockingSystem
# import communication.GUI_communication
from communication.GUI_communication import TCPIn
from communication.GUI_communication import TCPOut
from communication.GUI_communication import UDP
from communication.GUI_communication import serialCom
import config



if __name__ == "__main__":
    # Taking in connection details when running script in terminal
    parser = argparse.ArgumentParser(description="Ping python library example.")
    parser.add_argument('--device', action="store", required=True, type=str, help="Ping device port.")
    parser.add_argument('--baudrate', action="store", type=int, default=2000000, help="Ping device baudrate.")
    args = parser.parse_args()

    # Establishes connection to Ping 360 sonar
    p = Ping360()
    p.connect_serial(args.device, args.baudrate)    # Added this to get working

    # Initialization parameters
    print("Initialized: %s" % p.initialize())
    p.set_transmit_frequency(1000) # Original: 1000
    p.set_sample_period(50) # Original: 80            Sets sample period in ticks
    p.set_number_of_samples(1200) # Original: 500     Determines the resolution (higher num -> finer details)
    p.set_range(50) # Initial mode
    prevMode = -1

    # Initalize interlocking system for motor driving zones
    ils = InterlockingSystem()

    # Variables
    objectData = []
    operatorForceReset = False

    # TCP communication variables
    HOST = "169.254.226.72"  # The IP address of the RASPBERRY Pi assigns to this communication
    PORT = 1422  # Port to listen on (non-privileged ports are > 1023)
    HEADERSIZE = 10




    # Start a thread that handles communication with GUI
    #t1 = threading.Thread(target=communicationWithGUI, args=[HOST, PORT])
    #t1.start()

    """
    # Initialize serial communication with Arduino UNO
    ardSer = serial.Serial('/dev/ttyACM0', 9600, timeout=1,
    parity=serial.PARITY_NONE, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE)
    """
    SerialThread = threading.Thread(target=serialCom)
    SerialThread.start()

    print("[INFO] Serial communication with Arduino is initialized!")

    # Establishes a reliable data delivery TCP connection
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))    # Binds <eth0> port to requested IP and Port
    s.listen(2)             # Specifies number of unaccepted connection before refusing new

    """
    # Runs communication by sending and receiving data with GUI
    if config.address == "":
        print("[ATTENTION] Start computer script to initialize TCP communication with ROV!")

    TCPOut(s, HOST, PORT, HEADERSIZE)
    TCPThread = threading.Thread(target=TCPIn)
    TCPThread.start()
    """


    # Using UDP for camera transmission the camera will after stop and print <select timeout> to the console
    UDPThread = threading.Thread(target=UDP)
    UDPThread.start()


    print("OPENING CAMERA PORT")
    # cam = cv2.VideoCapture(0)   # Initializes connection to camera
    # vs = VideoStream(src=0).start()

    while 1:

        p.transmitAngle(config.angle)
        data = bytearray(getattr(p,'_data'))    # Fetches sonar echo strengths

        config.data_lst = []    # After sending last angle readings, reset for next iteration

        for k in data :
            config.data_lst.append(k)



        # If no TCP connection, try to establish one
        if config.address == "":
            print("[ATTENTION] Start computer script to initialize TCP communication with ROV!")
            TCPOut(s, HOST, PORT, HEADERSIZE)
            TCPThread = threading.Thread(target=TCPIn)
            TCPThread.start()

        # Updating system variables for communication with GUI
        config.step = p.get_step()
        config.interlockedZones = ils.lockedZones

        # Sends sensor and system data to GUI
        TCPOut(s, HOST, PORT, HEADERSIZE)



        # print(f'light value communicated through TCP {config.light}')

        if config.mode != prevMode:
            print("A new mode has been activated")
            p.changeOperatingMode(config.mode)
            prevMode = config.mode


        # print(f"The sonar is scanning from 0 to {round(p.get_range(), 2)} meters")
        # print(f"Set range is {p.get_range()}, set step is {p.get_step()}")

        if config.forceReset:
            print("Operator is forcing reset of all interlocked zones")
            ils.resetAllZones()


        # If object is found, interlock current zone
        if ils.findObject(config.data_lst):
            ils.setInterlockZone(ils.findZone(config.angle), config.angle)

        # Incrementing next scan, to not automatically reset the set zone above with the under statement
        config.angle = (config.angle + p.get_step()) % 400
        # print(f"Scanning angle {config.angle}")

        # If the current angle is equal to any of the angles that were used to lock a zone
        if ils.checkIfResetPermitted(config.angle):
            # Reset that zone as the sonar has scanned that zone again, and no object is detect
            ils.resetInterlockZone(ils.findZone(config.angle))

        # print(f"All currently locked zones: {ils.lockedZones}")
        # print(f"All zones locked angles: {ils.zoneLockedAngles}")
        # print("One loop in Python")
        # print("")