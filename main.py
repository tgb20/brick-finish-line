import cv2
import numpy as np

cam = cv2.VideoCapture(0)

first_frame = None

while True:
    check, frame = cam.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)

    if first_frame is None:
        first_frame = gray
        continue

    frame_delta = cv2.absdiff(first_frame, gray)
    thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=2)

    # Crop needs to be adjusted to fit finish line
    shape = thresh.shape
    height = shape[0]
    width = shape[1]
    half_width = int(width/2)

    left = frame[0:height, 0:half_width]
    right = frame[0:height, half_width:width]

    left_thresh = thresh[0:height, 0:half_width]
    right_thresh = thresh[0:height, half_width:width]


    contours_left, _ = cv2.findContours(image=left_thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(image=left, contours=contours_left, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

    contours_right, _ = cv2.findContours(image=right_thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(image=right, contours=contours_right, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)


    if(len(contours_left) >= 1):
        cv2.putText(left, 'Movement', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        # Send left track timer stop

    
    if(len(contours_right) >= 1):
        cv2.putText(right, 'Movement', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        # Send right track timer stop

    cv2.imshow('left', left)
    cv2.imshow('right', right)



    key = cv2.waitKey(1)
    if key == 27:
        break

cam.release()
cv2.destroyAllWindows()