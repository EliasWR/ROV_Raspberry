#!/usr/bin/env python3
import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import cv2

# Custom libraries imports
from algebra.multiplication import Multiplication
from sonarFunctionality.BlueRoboticsSonar import Ping360
from sonarFunctionality.Interlocking import InterlockingSystem

import json



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

    # Initalize interlocking system for omotor driving zones
    ils = InterlockingSystem()

    # Variables
    max_range = 80*200*1450/2
    step = 1
    length = 640
    image = np.zeros((length, length, 1), np.uint8)
    angle = 0
    objectData = []
    operatorForceReset = False


    # Serial communication with Arduino UNO
    # ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1) # OLD
    # ser.reset_input_buffer() # OLD

    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1, parity=serial.PARITY_NONE, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE)
    # Serial communication with Arduino UNO





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



        dataOut = {}
        number = 50
        number2 = 1
        dataOut["light"] = number%255 # 0 - 255 light settings
        dataOut["runZone"] = number2 # 1-8 Linear directions, 9 and 10 clock and counter-clock respectively
        dataOut["locked"] = [True, True, False, False, False, False, False, False] # 8 item array
        dataOut = json.dumps(dataOut)

        # If program takes longer to run, there might be problem with serial
        # Band-aid solution could be to add delay in Arduino C++ script
        if ser.in_waiting > 0:
            input_str = ser.readline().decode("utf-8").rstrip()
            print(input_str)
            ser.write(dataOut.encode())






        print(f"The sonar is scanning from 0 to {round(p.get_range(), 2)} meters")
        changeOperatingMode(2)
        print(f"Set range is {p.get_range()}, set step is {p.get_step()}")


        if operatorForceReset:
            ils.resetAllZones()

        # If object is found, interlock current zone
        if ils.findObject(data_lst):
            ils.setInterlockZone(ils.findZone(angle), angle)

        # Incrementing next scan, to not automatically reset the set zone above with the under statement
        angle = (angle + p.get_step()) % 400
        print(f"Scanning angle {angle}")

        # If the current angle is equal to any of the angles that were used to lock a zone
        if ils.checkIfResetPermitted(angle):
            # Reset that zone as the sonar has scanned that zone again, and no object is detect
            ils.resetInterlockZone(ils.findZone(angle))

        print(f"Zone 3 locked status: {ils.zoneLock3}")
        print(f"All zones locked angles: {ils.zoneLockedAngles}")
        print("")