import socket
import pickle    # Serialization of data for GUI communication
import cv2
import numpy as np
import config


def communicationWithGUI(s, cam, HOST, PORT, HEADERSIZE):
    communicating = True

    while communicating:
        receiving = True    # When beginning while loop we want to receive a message from GUI

        if not config.address: # If no address is given, try to find one
            config.clientsocket, config.address = s.accept()
            print(f"Connection from {config.address} has been established.")

        ret_val, img = cam.read()   # Takes a photo with camera
        compressed_img = commpressImage(img, 15)    # Compressing image

        # Finalizing dicitionary with all values to be sent to GUI
        GuiDataOut = {
            "image": compressed_img,
            "temp": config.temp,
            "pressure": config.pressure,
            "leak": config.leak,
            "angle": config.angle,
            "step": config.step,
            "lockedZones": config.interlockedZones,
            "dataArray": config.data_lst
        }

        # Serializing the dicitionary and sending
        msg = pickle.dumps(GuiDataOut)
        msg = bytes(f"{len(msg):<{HEADERSIZE}}", 'utf-8') + msg
        config.clientsocket.send(msg)

        # Receiving answer from computer
        while receiving:
            full_msg = b''
            new_msg = True
            while receiving:
                incoming_message = config.clientsocket.recv(8192)
                # print(incoming_message)
                if new_msg:    # Changed rom if new_msg
                    msglen = int(incoming_message[:HEADERSIZE])
                    new_msg = False

                full_msg += incoming_message

                if len(full_msg)-HEADERSIZE == msglen:
                    GuiDataIn = pickle.loads(full_msg[HEADERSIZE:])  # Deserialize reponse from GUI

                    # Setting the global variables accordingly
                    config.light = GuiDataIn["light"]
                    config.runZone = GuiDataIn["runZone"]
                    config.mode = GuiDataIn["mode"]
                    config.forceReset = GuiDataIn["forceReset"]

                    # Resetting variables for next iteration
                    receiving = False
                    new_msg = True
                    full_msg = b''

                    communicating = False   # Exits the function

def commpressImage(img, k):
    width = int((img.shape[1])/k)
    height = int((img.shape[0])/k)
    return cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)