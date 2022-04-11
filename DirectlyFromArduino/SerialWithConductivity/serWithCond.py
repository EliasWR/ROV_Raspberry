from xml.dom import UserDataHandler
from cv2 import split
import serial
import time
import sys

ser = serial.Serial('/dev/ttyUSB0', 9600)   # Conductivity sensor got selected as USB port 0, therefore -> Sonar USB port 1 ttyUSB1
time.sleep(2)
# ser.write("set_interval(3)\n".encode())
print(ser.isOpen())
# throwawayInput = ser.readline()
i = 0


while 1:
    # Tells sensor to initiate a sensor reading
    print("Entered while loop")
    ser.write("do_sample\n".encode())
    print("This line here")
    input = ser.readline()  # Have to read the buffer to stop future splitting issues
    ser.write("get_salinity\n".encode())
    input = ser.readline().decode()
    # input = input.decode()
    salinityReading = input.split('\t')

    input = ser.readline()  # Have to read the buffer to stop future splitting issues
    ser.write("get_soundspeed\n".encode())
    input = ser.readline().decode()
    soundSpeedReading = input.split('\t')

    input = ser.readline()  # Have to read the buffer to stop future splitting issues
    ser.write("get_density\n".encode())
    input = ser.readline().decode()
    densityReading = input.split('\t')

    input = ser.readline()  # Have to read the buffer to stop future splitting issues
    ser.write("get_conductivity\n".encode())
    input = ser.readline().decode()
    conductivityReading = input.split('\t')

    print(f'Iteration number {i}')
    print(f'Salinity = {salinityReading[-1]}')
    print(f'Sound of speed: {soundSpeedReading[-1]}')
    print(f'Conductivity = {conductivityReading[-1]}')
    print(f'density: {densityReading[-1]}')
    # print(f'Soundspeed = {splitted[-1]}')
    # print(splitted[0])
    # print(splitted[1])

    input = ser.readline()

"""
COMMANDS THAT NEEDS TO BE USED
when initial connection with sensor
turn off auto sampling
>>stop

Before every reading of value
>>do_sample     (Tells the sensor to read all units)
get_pressure    [kPa]
get_salinity    [PSU (Practicial salinity unit)] 1PSU = 1 g/kg
get_soundspeed  [m/s]
get_density     [kg/m^3]
get_conductivity [mS/cm]

"""