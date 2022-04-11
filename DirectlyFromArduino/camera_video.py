import cv2
import sys
from imutils.video import VideoStream

vs = VideoStream(src=0).start()



i = 0

while 1:
# _, frame = cap.read()
    i += 1
    print(i)
    frame = vs.read()

    frame = cv2.resize(frame, None, fx = 0.4, fy = 0.4, interpolation=cv2.INTER_AREA)

    print(type(frame))
    print(sys.getsizeof(frame))

    cv2.imshow('Input', frame)
    c = cv2.waitKey(1)
    if c == 27:
        break




cap.release()
cv2.destroyAllWindows()
s.close()