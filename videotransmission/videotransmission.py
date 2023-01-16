import cv2
import numpy

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    # print(ret)
    cv2.imshow('capture', frame)
    asc = cv2.waitKey(1)
    if asc == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

