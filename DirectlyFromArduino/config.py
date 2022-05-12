##  Initializing and setting default values for global TCP data objects
# From GUI
light= 0
motorSpeed = 0
runZone = -1
mode = 1
forceReset = False
takeHighResPhoto = False
takeVideo = False

# To GUI
# image
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


# General functionality
address = ""
clientsocket = ""
newArduinoCommands = False