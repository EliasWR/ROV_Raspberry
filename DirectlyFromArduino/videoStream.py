from imutils.video import VideoStream
import imutils
import imagezmq
import socket
import time
import argparse
import cv2
import sys


vs = VideoStream(src=0).start()
# vs = .stream.set(3, 640)
# vs = .stream.set(4, 480)

while 1:
    frame = vs.read()
    frame = imutils.resize(frame, width=400)
    print(sys.getsizeof(frame))
    cv2.imshow('Input', frame)
    c = cv2.waitKey(1)
    if c == 27:
        break
