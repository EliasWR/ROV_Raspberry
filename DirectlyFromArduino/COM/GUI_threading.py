import socket
import pickle    # Serialization of data for GUI communication
import cv2
import numpy as np
import config


def communicationWithGUI(HOST, PORT):

    communicating = True

    """ LOGIC BELONGING TO THREADING
    HEADERSIZE = 10
    address = ""


    # Establishes a reliable and in-order data delivery TCP connection
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))    # Binds <eth0> port to requested IP and Port
    s.listen(2)             # Specifies number of unaccepted connection before refusing new

    print("OPENING CAMERA PORT")
    cam = cv2.VideoCapture(0)   # Initializes connection to camera
    """

    while communicating:
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
                    # Setting the global variables accordingly
                    config.x = response["key1"]
                    print(f'X value inside thread {config.x}')

                    # Resetting variables for next iteration
                    receiving = False
                    new_msg = True
                    full_msg = b''

                    communicating = False   # Exits the function


def commpressImage(img, k):
    width = int((img.shape[1])/k)
    height = int((img.shape[0])/k)
    return cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)