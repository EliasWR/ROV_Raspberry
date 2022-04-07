import cv2
from math import *


"""
Variables that are needed to plot:


data_lst = []
angle
get_step()
max_range = 80*200*1450/2   # Might have to be found from sonar class
length = 640
image = np.zeros((length, length, 1), np.uint8)
"""


while 1:
    center = (length/2,length/2)
    linear_factor = len(data_lst)/center[0]
    for i in range(int(center[0])):
        if(i < center[0]*max_range/max_range):
            try:
                pointColor = data_lst[int(i*linear_factor-1)]
            except IndexError:
                pointColor = 0

        else:
            pointColor = 0
        for k in np.linspace(0,p.get_step(),8*p.get_step()):
            image[int(center[0]+i*cos(2*pi*(angle+k)/400)), int(center[1]+i*sin(2*pi*(angle+k)/400)), 0] = pointColor

    color = cv2.applyColorMap(image,cv2.COLORMAP_JET)
    cv2.imshow('Sonar Image',image)
    cv2.waitKey(25)