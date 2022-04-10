"""
    UDP COMMUNICATION WITH GUI TEST FILE
"""

import socket
import cv2
import sys
import pickle

localIP = "169.254.226.72"
localPort = 20001
bufferSize = 1024

msgFromServer = "HELLO HEO HO"
bytesToSend = str.encode(msgFromServer)

UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

UDPServerSocket.bind((localIP, localPort))

print("UDP SERVER IS LISTENING FOR INPUT")

cap = cv2.VideoCapture(0)

bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
message = bytesAddressPair[0]
address = bytesAddressPair[1]
print(f"Message from client: {message}")
print(f"Client IP address: {address}")

while 1:
    ret, frame = cap.read()
    frame = cv2.resize(frame, None, fx = 0.06, fy = 0.06, interpolation=cv2.INTER_AREA)

    camera_dict = {
        "ret": ret,
        "frame": frame
    }

    camera_dict = pickle.dumps(camera_dict)

    print(sys.getsizeof(camera_dict))

    UDPServerSocket.sendto(camera_dict, address)











"""
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    raise IOError("Cannot open webcam")
i = 0

while True:
    ret, frame = cap.read()
    frame = cv2.resize(frame, None, fx = 0.3, fy = 0.3, interpolation=cv2.INTER_AREA)
    print(type(frame))
    print(sys.getsizeof(frame))
    i += 1
    print(i)
    cv2.imshow('Input', frame)
    c = cv2.waitKey(1)
    if c == 27:
        break

cap.release()
cv2.destroyAllWindows()
"""