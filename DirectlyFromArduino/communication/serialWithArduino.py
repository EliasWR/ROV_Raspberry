import serial
import time
import json
import time

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1, parity=serial.PARITY_NONE, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE)

number = 0
number2 = 1
# input_str = ser.readline()

while True:



    dataOut = {}
    number = 20
    number2 = number2 + 1
    dataOut["light"] = 100 # 0 - 255 light settings
    dataOut["runZone"] = 3 # 1-8 Linear directions, 9 and 10 clock and counter-clock respectively
    dataOut["locked"] = [False, False, False, False, False, False, False, False] # 8 item array
    dataOut = json.dumps(dataOut)



    # print(dataOut)
    """
    input_str = ser.readline().decode("utf-8").rstrip()
    if (input_str == ""):

        print("Input string is empty")
    else:
        try:
            print(input_str)
            # data = json.loads(input_str)
            # print(data)
            # print(data["Temp"])
            # print(data["Pressure"])
            # print(data["Leak"])
            ser.write(dataOut.encode('ascii'))
        except:
            print("An error when decoding from JSON occurred")
    """

    if ser.in_waiting > 0:
        input_str = ser.readline().decode("utf-8").rstrip()
        print(input_str)
        ser.write(dataOut.encode())

