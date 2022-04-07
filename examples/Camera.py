import cv2
import sys

cap = cv2.VideoCapture(0)



if not cap.isOpened():
    raise IOError("Cannot open webcam")
i = 0

while True:
    ret, frame = cap.read()
    frame = cv2.resize(frame, None, fx = 0.5, fy = 0.5, interpolation=cv2.INTER_AREA)
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