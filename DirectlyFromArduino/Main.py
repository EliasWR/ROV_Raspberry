#!/usr/bin/env python3
"""
RASPBERRY PI MAIN PROGRAM FOR ROV OPERATION. PROGRAM USES MULTIPLE
PYTHON SUB SCRIPTS FOR COMMUNICATION, COLLISION AVOIDANCE AND GENERAL
SENSOR READINGS.
"""

import numpy as np
import threading
from math import *
import socket
import argparse

# Custom libraries imports for specific functionality
from sonarFunctionality.BlueRoboticsSonar import Ping360
from sonarFunctionality.Interlocking import InterlockingSystem
from COM.communication import TCPIn
from COM.communication import TCPOut
from COM.communication import UDP
from COM.communication import serialCom
import config


if __name__ == "__main__":
    # Terminal connection alternatives for sonar connection
    parser = argparse.ArgumentParser(description="Ping python library example.")
    parser.add_argument('--device', action="store", required=True, type=str, help="Ping device port.")
    parser.add_argument('--baudrate', action="store", type=int, default=2000000, help="Ping device baudrate.")
    args = parser.parse_args()

    # Establishes connection to Ping 360 sonar
    p = Ping360()
    p.connect_serial(args.device, args.baudrate) 

    # Defining sonar parameters
    print("Initialized: %s" % p.initialize())
    p.set_transmit_frequency(1000) 
    p.set_sample_period(50)
    p.set_number_of_samples(1200)
    p.set_range(50)

    # Initial zone control command to stand-still thrusters 
    prevMode = -1

    # Initalize interlocking system for motor driving zones
    ils = InterlockingSystem()

    # Variables for internal logic
    objectData = []
    operatorForceReset = False

    # TCP communication variables
    HOST = "169.254.226.72"  # The IP address of the RASPBERRY Pi assigns to this communication
    PORT = 1422  # Port to listen on (non-privileged ports are > 1023)
    HEADERSIZE = 10

    # Establishes a reliable data delivery TCP connection
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))    # Binds <eth0> port to requested IP and Port
    s.listen(2)             # Specifies number of unaccepted connection before refusing new

    # Initialize serial communication with seperate thread
    SerialThread = threading.Thread(target=serialCom)
    SerialThread.start()

    # Initialize UDP communication with seperate thread
    UDPThread = threading.Thread(target=UDP)
    UDPThread.start()

    # Continuously running while loop handling communication and various commands
    while 1:

        # Sets new angle for sonar to scan
        p.transmitAngle(config.angle)
        
        # Reads sonar echo strengths into array for one angle
        data = bytearray(getattr(p,'_data')) 

        # Empties sonar data from previous iteration from array
        config.data_lst = []    

        # Stores echo strengths in global variable
        for k in data :
            config.data_lst.append(k)

        # If no TCP connection already established, attempt to establish
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

        # If new command for mode control is received, perform changes
        if config.mode != prevMode:
            print("A new mode has been activated")
            p.changeOperatingMode(config.mode)
            prevMode = config.mode

        # Checks for operator induced forced reset of interlocked zones
        if config.forceReset:
            print("Operator is forcing reset of all interlocked zones")
            ils.resetAllZones()

        # If object is found, interlock the current zone
        if ils.findObject(config.data_lst):
            ils.setInterlockZone(ils.findZone(config.angle), config.angle)

        # Incrementing for next sonar-scan
        config.angle = (config.angle + p.get_step()) % 400

        # If the current angle is equal to any of the angles that were used to lock a zone
        if ils.checkIfResetPermitted(config.angle):
            # Reset that zone as the sonar has scanned that zone again, and no object is detect
            ils.resetInterlockZone(ils.findZone(config.angle))
