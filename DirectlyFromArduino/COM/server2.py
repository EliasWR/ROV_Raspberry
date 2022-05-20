import socket
import time
import pickle
import numpy as np
import cv2
import sys

HEADERSIZE = 10

HOST = "169.254.226.72"  # The IP address of the RASPBERRY Pi assigns to this communication
PORT = 65432  # Port to listen on (non-privileged ports are > 1023)
address = ""

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(5)

# Open CV capture a single frame
cam = cv2.VideoCapture(0)


while True:

    receiving = True    # Will enter reading logic after sending of image is completed

    if not address: # Added to prevent looking for new address for following iterations
        clientsocket, address = s.accept()
        print(f"Connection from {address} has been established.")


    d = np.array([[2,2],[2,2]])
    ret_val, img = cam.read()
    # Compressing image
    k = 5   # k = 5 -> 603kB k = 20 -> 37.6kB
    width = int((img.shape[1])/k)
    height = int((img.shape[0])/k)
    scaled = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)

    d = scaled
    # print(sys.getsizeof(d))
    msg = pickle.dumps(d)   # Serializing the np array object to sendable form
    msg = bytes(f"{len(msg):<{HEADERSIZE}}", 'utf-8')+msg
    # print(sys.getsizeof(msg))
    # print(len(msg)) # 15 116 7118 bytes before compression
    # print(msg)
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
                receiving = False   # After reading is complete a new sending sequence can begin
                # print(f"[RECEIVED] message from GUI / {address}")

                response = pickle.loads(full_msg[HEADERSIZE:])
                # print(response)

                # Resetting variables for next iteration
                new_msg = True
                full_msg = b''










