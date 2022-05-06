import cv2
import sys

cap = cv2.VideoCapture(0)
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
size = (frame_width, frame_height)
result = cv2.VideoWriter('/home/pi/Programs/Videos/myVideo6.avi',
cv2.VideoWriter_fourcc(*'mp4v'), 1, size)


if not cap.isOpened():
    raise IOError("Cannot open webcam")
i = 0

while True:
    ret, frame = cap.read()
    result.write(frame)
    frame = cv2.resize(frame, None, fx = 0.3, fy = 0.3, interpolation=cv2.INTER_AREA)

    i += 1

    cv2.imshow('Input', frame)
    c = cv2.waitKey(1)
    if c == 27:
        break

cap.release()
result.release()
cv2.destroyAllWindows()
print("saved")