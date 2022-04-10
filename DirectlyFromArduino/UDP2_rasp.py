import cv2
import socket
import struct
import numpy as np
import math
from imutils.video import VideoStream
import time

class FrameSegment(object):
  """“””
  Object to break down image frame segment
  if the size of image exceed maximum datagram size
  “””"""

  def __init__(self, sock, port, addr="169.254.226.73"):
    self.s = sock
    self.port = port
    self.addr = addr
    self.MAX_DGRAM = 2**16
    self.MAX_IMAGE_DGRAM = self.MAX_DGRAM - 64 # minus 64 bytes in case UDP frame overflown

  def udp_frame(self, img):
    """“””
    Compress image and Break down
    into data segments
    “””"""
    compress_img = cv2.resize(img, None, fx = 0.4, fy=0.4, interpolation=cv2.INTER_AREA)
    compress_img = cv2.imencode(".jpg", compress_img)[1]

    dat = compress_img.tostring()
    size = len(dat)
    num_of_segments = math.ceil(size/(self.MAX_IMAGE_DGRAM))
    array_pos_start = 0

    while num_of_segments:
      array_pos_end = min(size, array_pos_start + self.MAX_IMAGE_DGRAM)
      self.s.sendto(
                   struct.pack("B", num_of_segments) +
                   dat[array_pos_start:array_pos_end],
                   (self.addr, self.port)
                   )
      array_pos_start = array_pos_end
      num_of_segments -= 1




""" “”” Top level main function “””"""
# Set UDP socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
port = 20001
fs = FrameSegment(s, port)

# cap = cv2.VideoCapture(0)
vs = VideoStream(src=0).start()



while 1:
    # _, frame = cap.read()
    frame = vs.read()
    fs.udp_frame(frame)


cap.release()
cv2.destroyAllWindows()
s.close()