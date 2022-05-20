"""
RASPBERRY PI SUB PROGRAM CONTAINING GLOBAL VARIABLES USED BETWEEN
THE OTHER PYTHON SCRIPTS.
"""

# Global variables being received from GUI
light= 0
motorSpeed = 0
runZone = -1
mode = 1
forceReset = False
takeHighResPhoto = False
takeVideo = False

# Global variables being sent to GUI
temp = 0
depth = 0
leak = False
angle = 0
data_lst = []
step = 0
interlockedZones = [False] * 8
salinity = 0
conductivity = 150
density = 1000

# General functionality not used by communication
address = ""
clientsocket = ""
newArduinoCommands = False