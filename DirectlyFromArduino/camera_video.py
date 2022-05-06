import cv2
import sys
from imutils.video import VideoStream

vs = VideoStream(src=0).start()




while 1:

    frame = vs.read()

    # frame = cv2.resize(frame, None, fx = 0.4, fy = 0.4, interpolation=cv2.INTER_AREA)

    cv2.imshow('Input', frame)
    c = cv2.waitKey(1)
    if c == 27:
        break




cap.release()
cv2.destroyAllWindows()
s.close()