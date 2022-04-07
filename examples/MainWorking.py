
#!/usr/bin/env python3
import serial
import time
import numpy as np
# import matplotlib.pyplot as plt
# import cv2

# Custom libraries imports
from sonarFunctionality.BlueRoboticsSonar import Ping360
from sonarFunctionality.Interlocking import InterlockingSystem
# import communication.GUI_communication
# from communication.GUI_communication import communicationWithGUI
import config


import json # Serialization of data for Arduino communication
import threading




if __name__ == "__main__":
    import argparse
    import cv2
    import numpy as np
    from math import *

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
    p.set_range(50) # Range on start up

    # Initalize interlocking system for motor driving zones
    ils = InterlockingSystem()

    # Variables
    max_range = 80*200*1450/2
    length = 640
    image = np.zeros((length, length, 1), np.uint8)
    angle = 0
    objectData = []
    operatorForceReset = False

    # Global variables used in communication
    #x = 10
    #y = 20

    # TCP communication variables
    HOST = "169.254.226.72"  # The IP address of the RASPBERRY Pi assigns to this communication
    PORT = 1422  # Port to listen on (non-privileged ports are > 1023)


    import socket
    import pickle    # Serialization of data for GUI communication
    import cv2
    import numpy as np


    def communicationWithGUI(HOST, PORT):

        HEADERSIZE = 10
        address = ""

        # Establishes a reliable and in-order data delivery TCP connection
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((HOST, PORT))    # Binds <eth0> port to requested IP and Port
        s.listen(2)             # Specifies number of unaccepted connection before refusing new

        print("OPENING CAMERA PORT")
        cam = cv2.VideoCapture(2)   # Initializes connection to camera


        while True:
            receiving = True    # When beginning while loop we want to receive a message from GUI

            if not address: # If no address is given, try to find one
                clientsocket, address = s.accept()
                print(f"Connection from {address} has been established.")

            ret_val, img = cam.read()   # Takes a photo with camera
            compressed = commpressImage(img, 10)    # Compressing image

            # Finalizing dicitionary with all values to be sent to GUI
            d = {
                "key1": compressed,
                "key2": 232,
                "key3": [True, False, True],
                "key4": 23
            }

            # Serializing the dicitionary and sending
            msg = pickle.dumps(d)
            msg = bytes(f"{len(msg):<{HEADERSIZE}}", 'utf-8') + msg
            clientsocket.send(msg)


            # Receiving answer from computer
            while receiving:
                full_msg = b''
                new_msg = True
                while receiving:
                    incoming_message = clientsocket.recv(20000)
                    # print(incoming_message)
                    if new_msg:    # Changed rom if new_msg
                        msglen = int(incoming_message[:HEADERSIZE])
                        new_msg = False

                    full_msg += incoming_message

                    if len(full_msg)-HEADERSIZE == msglen:
                        response = pickle.loads(full_msg[HEADERSIZE:])  # Deserialize reponse from GUI
                        global x
                        global y
                        # Setting the global variables accordingly
                        x = response["key1"]
                        print(f'X value inside thread {x}')

                        # Resetting variables for next iteration
                        receiving = False
                        new_msg = True
                        full_msg = b''


    def commpressImage(img, k):
        width = int((img.shape[1])/k)
        height = int((img.shape[0])/k)
        return cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)




    def changeOperatingMode(mode):
        # Checks if input is different from last iteration
        if mode == 0:
            p.set_range(2) # Short range collision avoidance mode
            p.set_step(8)
            p.set_gain_setting(0)
        elif mode == 1:
            p.set_range(4) # Medium range collision avoidance mode
            p.set_step(4)
            p.set_gain_setting(0)
        elif mode == 2:
            p.set_range(50) # Aquaculture inspection mode
            p.set_step(2)
            p.set_gain_setting(1)
        else:
            print("Did not recognize mode command")
            print("Corrupt or invalid data given")
            return

    # Start a thread that handles communication with GUI
    t1 = threading.Thread(target=communicationWithGUI, args=[HOST, PORT])
    t1.start()

    # Initialize serial communication with Arduino UNO
    ardSer = serial.Serial('/dev/ttyACM1', 9600, timeout=1,
    parity=serial.PARITY_NONE, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE)

    print("CONNECTION WITH ARDUINO ESTABLISED")


    while(True):
        theta_ = []
        r_ = []
        color_ = []
        p.transmitAngle(angle)
        data = bytearray(getattr(p,'_data'))
        data_lst = []

        # print(f"{len(data)} number of values in data")

        for k in data :
            data_lst.append(k)

        # Drastic speed up when disabling the plotting of scanned map, have to handle this on surface
        """
        center = (length/2,length/2)
        linear_factor = len(data_lst)/center[0]
        for i in range(int(center[0])):
            if(i < center[0]*max_range/max_range):
                try:
                    pointColor = data_lst[int(i*linear_factor-1)]
                except IndexError:
                    pointColor = 0

            else:
                pointColor = 0
            for k in np.linspace(0,p.get_step(),8*p.get_step()):
                image[int(center[0]+i*cos(2*pi*(angle+k)/400)), int(center[1]+i*sin(2*pi*(angle+k)/400)), 0] = pointColor

        color = cv2.applyColorMap(image,cv2.COLORMAP_JET)
        cv2.imshow('Sonar Image',image)
        cv2.waitKey(25)
        """

        # SERIAL WITH ARDUINO
        ArdDataOut = {}
        print(f'X value outside thread {config.x}')
        ArdDataOut["light"] = 1 # SET BY GUI: 0-255 light settings
        ArdDataOut["runZone"] = 1 # SET BY GUI: 1-8 Linear directions, 9 and 10 clock and counter-clock respectively
        ArdDataOut["locked"] = ils.lockedZones
        ArdDataOut = json.dumps(ArdDataOut)

        # If program takes longer to run, there might be problem with serial
        # Band-aid solution could be to add delay in Arduino C++ script
        print("One loop in Python")
        if ardSer.in_waiting > 0:
            input_str = ardSer.readline().decode("utf-8").rstrip()
            print(input_str)
            print(ArdDataOut)
            ardSer.write(ArdDataOut.encode())




        changeOperatingMode(0)  # SET BY GUI
        operatorForceReset = False # SET BY GUI
        # print(f"The sonar is scanning from 0 to {round(p.get_range(), 2)} meters")
        # print(f"Set range is {p.get_range()}, set step is {p.get_step()}")


        if operatorForceReset:
            ils.resetAllZones()

        # If object is found, interlock current zone
        if ils.findObject(data_lst):
            ils.setInterlockZone(ils.findZone(angle), angle)

        # Incrementing next scan, to not automatically reset the set zone above with the under statement
        angle = (angle + p.get_step()) % 400
        # print(f"Scanning angle {angle}")

        # If the current angle is equal to any of the angles that were used to lock a zone
        if ils.checkIfResetPermitted(angle):
            # Reset that zone as the sonar has scanned that zone again, and no object is detect
            ils.resetInterlockZone(ils.findZone(angle))


        # print(f"All currently locked zones: {ils.lockedZones}")
        # print(f"All zones locked angles: {ils.zoneLockedAngles}")
        print("")