from cv2 import cv2
import time

capture = cv2.VideoCapture(0)

time.sleep(0.1)

while True:
    ret, frame = capture.read()
    cv2.imshow("Video", frame)

    if cv2.waitKey(20) & 0xFF == ord('d'):
        break


cv2.destroyAllWindows()