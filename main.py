import cv2
import numpy as np
import serial
from time import time

cam = cv2.VideoCapture(0)

first_frame = None

ser = serial.Serial(
    port='/dev/cu.usbmodem1424401',
    baudrate = 9600,
    timeout=1
)

# reset the timers
ser.write(0)

status = 'reset'
race = True
start_time = 0
left_time = 0
right_time = 0

while race:
    line = ser.readline() 
    if line:
        status = line.decode().strip()

        while status == 'go':
            if start_time == 0:
                start_time = int(time() * 1000)

            check, frame = cam.read()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)

            if first_frame is None:
                first_frame = gray
                continue

            frame_delta = cv2.absdiff(first_frame, gray)
            thresh = cv2.threshold(frame_delta, 75, 255, cv2.THRESH_BINARY)[1]
            thresh = cv2.dilate(thresh, None, iterations=4)

            # Crop needs to be adjusted to fit finish line
            shape = thresh.shape
            height = shape[0]
            width = shape[1]
            half_width = int(width/2)
            quarter_width = int(half_width/2)
            framed_height = int(height/3)
            left_height = int(height - framed_height)

            left = frame[left_height:height, quarter_width:half_width]
            right = frame[left_height:height, half_width:width-quarter_width]

            left_thresh = thresh[0:height, 0:half_width]
            right_thresh = thresh[0:height, half_width:width]

            contours_left, _ = cv2.findContours(image=left_thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
            cv2.drawContours(image=left, contours=contours_left, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

            contours_right, _ = cv2.findContours(image=right_thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
            cv2.drawContours(image=right, contours=contours_right, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

            if(len(contours_left) >= 1 and left_time == 0):
                # Send left track timer stop 
                ser.write(bytes('1', 'utf-8'))
                left_time = int(time() * 1000) - start_time
                # write to database
     
            if(len(contours_right) >= 1 and right_time == 0):
                # Send right track timer stop
                ser.write(bytes('2', 'utf-8'))
                right_time = int(time() * 1000) - start_time
                # write to database

            if left_time != 0:
                cv2.putText(left, str(left_time), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA) 

            if right_time != 0:
                cv2.putText(right, str(right_time), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

            cv2.imshow('left', left)
            cv2.imshow('right', right)

            # cv2.imshow('thresh', thresh)

            key = cv2.waitKey(1)
            if key == 27:
                # ESC to quit
                race = False
                break 
            elif key == 32:
                # SPACE to reset
                status = 'reset'
                start_time = 0
                left_time = 0
                right_time = 0
                first_frame = gray
                ser.write(bytes('0', 'utf-8'))

cam.release()
cv2.destroyAllWindows()
ser.close()