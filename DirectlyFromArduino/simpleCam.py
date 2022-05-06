import cv2
import time

cap = cv2.VideoCapture(0)
#






if cap.isOpened():
    raise IOError("Cannot open webcam")

while 1:
    ret, frame = cap.read()
    # frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    cv2.imshow("input", frame)

    c = cv2.waitkey(1)
    if c == 27:
        break

cap.release()
cv2.destroyAllWindows()