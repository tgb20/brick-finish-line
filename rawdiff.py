from time import time
import cv2
import numpy as np
import serial


def frame_difference(frame1, frame2):
    # Convert frames to grayscale
    gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    # Calculate absolute difference between frames
    diff = cv2.absdiff(gray1, gray2)

    # Calculate percentage of different pixels
    total_pixels = diff.shape[0] * diff.shape[1]

    # Pixels greater than 30 are considered different
    _, diff = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)
    different_pixels = np.count_nonzero(diff)

    percentage = (different_pixels / total_pixels) * 100

    return percentage


# Initialize video capture
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 60)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

cv2.waitKey(1000)

left_first_frame = None
right_first_frame = None
ser = serial.Serial(port="/dev/cu.usbmodemDC5475C4BB602", baudrate=9600, timeout=1)

race_started = False
gate_reclosed = True
left_time = 0
right_time = 0

while True:
    # Read current frame
    _, current_frame = cap.read()

    line = ser.readline().decode().strip()

    # line == 1 means the gate is open
    # line == 0 means the gate is closed

    # Start the race when the gate is open but don't let a new race start until its closed again
    if line == "1" and gate_reclosed and not race_started:
        start_time = int(time() * 1000)
        gate_reclosed = False
        race_started = True
        left_time = 0
        right_time = 0
        print("Race Started")

    if line == "0" and not gate_reclosed:
        gate_reclosed = True

    left_frame = current_frame[0:200, 0:200]
    right_frame = current_frame[0:200, 200:400]

    # # If first frame is not set, set it and skip to next iteration
    if left_first_frame is None:
        left_first_frame = left_frame
        continue

    if right_first_frame is None:
        right_first_frame = right_frame
        continue

    # Calculate and print difference percentage
    left_diff_percentage = frame_difference(left_first_frame, left_frame)
    right_diff_percentage = frame_difference(right_first_frame, right_frame)

    if left_diff_percentage > 10 and race_started and left_time == 0:
        # Current time
        current_time = int(time() * 1000)
        left_time = current_time - start_time
        # print(f"Left Difference: {left_diff_percentage:.2f}%")
        print(f"Left Time: {left_time}ms")

    if right_diff_percentage > 10 and race_started and right_time == 0:
        # Current time
        current_time = int(time() * 1000)
        right_time = current_time - start_time
        # print(f"Right Difference: {right_diff_percentage:.2f}%")
        print(f"Right Time: {right_time}ms")

    if right_time != 0 and left_time != 0 and race_started:
        race_started = False
        print("Race over")

    # Display the current frame
    cv2.imshow("Current Left Frame", left_frame)

    cv2.imshow("Current Right Frame", right_frame)

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release the capture and close windows
cap.release()
cv2.destroyAllWindows()
ser.close()
