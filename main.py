from time import time
import cv2
import numpy as np
import serial

from PyQt5 import QtCore, QtGui, QtWidgets
import sys


class Window(QtWidgets.QMainWindow):
    def setupUi(self, MainWindow):

        self.status = 'reset'
        self.startTime = 0
        self.leftTime = 0
        self.rightTime = 0

        MainWindow.setObjectName('BrickTimer')
        MainWindow.resize(498, 522)

        # Primary Layout
        layout = QtWidgets.QVBoxLayout()

        # Top most bar
        barLayout = QtWidgets.QHBoxLayout()

        # Enter top bar
        self.startButton = QtWidgets.QPushButton('Start')
        self.startButton.clicked.connect(self.showCamera)
        barLayout.addWidget(self.startButton)

        self.enableSerial = QtWidgets.QCheckBox()
        self.enableSerial.setText('Enable Serial')
        barLayout.addWidget(self.enableSerial)

        topBar = QtWidgets.QWidget()
        topBar.setLayout(barLayout)

        layout.addWidget(topBar)
        # Leave top bar

        # Enter slider area
        slidersLayout = QtWidgets.QHBoxLayout()

        # Enter left controls
        leftControlsLayout = QtWidgets.QVBoxLayout()

        leftLabel = QtWidgets.QLabel('Left Controls')
        leftControlsLayout.addWidget(leftLabel)

        # Left Width Slider
        leftWidthLabel = QtWidgets.QLabel('Left Width')
        leftControlsLayout.addWidget(leftWidthLabel)
        self.leftWidth = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.leftWidth.setValue(50)
        leftControlsLayout.addWidget(self.leftWidth)

        # Left Height Slider
        leftHeightLabel = QtWidgets.QLabel('Left Height')
        leftControlsLayout.addWidget(leftHeightLabel)
        self.leftHeight = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.leftHeight.setValue(50)
        leftControlsLayout.addWidget(self.leftHeight)

        # Left X Slider
        leftXLabel = QtWidgets.QLabel('Left X')
        leftControlsLayout.addWidget(leftXLabel)
        self.leftX = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.leftX.setValue(50)
        leftControlsLayout.addWidget(self.leftX)

        # Left Y Slider
        leftYLabel = QtWidgets.QLabel('Left Y')
        leftControlsLayout.addWidget(leftYLabel)
        self.leftY = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.leftY.setValue(50)
        leftControlsLayout.addWidget(self.leftY)

        leftControls = QtWidgets.QWidget()
        leftControls.setLayout(leftControlsLayout)
        slidersLayout.addWidget(leftControls)
        # Leave left controls

        # Enter right controls
        rightControlsLayout = QtWidgets.QVBoxLayout()

        rightLabel = QtWidgets.QLabel('Right Controls')
        rightControlsLayout.addWidget(rightLabel)

        # Right Width Slider
        rightWidthLabel = QtWidgets.QLabel('Right Width')
        rightControlsLayout.addWidget(rightWidthLabel)
        self.rightWidth = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.rightWidth.setValue(50)
        rightControlsLayout.addWidget(self.rightWidth)

        # Right Height Slider
        rightHeightLabel = QtWidgets.QLabel('Right Height')
        rightControlsLayout.addWidget(rightHeightLabel)
        self.rightHeight = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.rightHeight.setValue(50)
        rightControlsLayout.addWidget(self.rightHeight)

        # Right X Slider
        rightXLabel = QtWidgets.QLabel('Right X')
        rightControlsLayout.addWidget(rightXLabel)
        self.rightX = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.rightX.setValue(50)
        rightControlsLayout.addWidget(self.rightX)

        # Right Y Slider
        rightYLabel = QtWidgets.QLabel('Right Y')
        rightControlsLayout.addWidget(rightYLabel)
        self.rightY = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.rightY.setValue(50)
        rightControlsLayout.addWidget(self.rightY)

        rightControls = QtWidgets.QWidget()
        rightControls.setLayout(rightControlsLayout)
        slidersLayout.addWidget(rightControls)
        # Leave right controls

        sliderArea = QtWidgets.QWidget()
        sliderArea.setLayout(slidersLayout)
        layout.addWidget(sliderArea)
        # Leave slider area

        # Threshold controls
        thresholdLayout = QtWidgets.QHBoxLayout()

        thresholdLabel = QtWidgets.QLabel('Threshold')
        thresholdLayout.addWidget(thresholdLabel)

        minLabel = QtWidgets.QLabel('Min')
        thresholdLayout.addWidget(minLabel)
        self.min = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.min.setRange(0, 255)
        self.min.setValue(75)
        thresholdLayout.addWidget(self.min)

        maxLabel = QtWidgets.QLabel('Max')
        thresholdLayout.addWidget(maxLabel)
        self.max = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.max.setRange(0, 255)
        self.max.setValue(255)
        thresholdLayout.addWidget(self.max)

        self.showThresh = QtWidgets.QCheckBox()
        self.showThresh.setText('Show Thresh')
        thresholdLayout.addWidget(self.showThresh)

        thresholdControls = QtWidgets.QWidget()
        thresholdControls.setLayout(thresholdLayout)
        layout.addWidget(thresholdControls)
        # Leave threshold controls

        # Primary Layout (renders video)
        self.label = QtWidgets.QLabel()
        layout.addWidget(self.label)

        widget = QtWidgets.QWidget()
        widget.setLayout(layout)
        MainWindow.setCentralWidget(widget)

    def showCamera(self):

        self.status = 'reset'
        self.startTime = 0
        self.leftTime = 0
        self.rightTime = 0

        ser = None

        if(self.enableSerial.checkState() == 2):
            ser = serial.Serial(
                port='/dev/cu.usbmodem1414401',
                baudrate=9600,
                timeout=1
            )

        if(ser):
            print("Reset serial")
            ser.write(bytes('0', 'utf-8'))

        self.startButton.setText('Reset')
        cam = cv2.VideoCapture(1)

        check, frame = cam.read()

        frameHeight, frameWidth, _ = frame.shape
        self.leftWidth.setRange(0, frameWidth)
        self.leftHeight.setRange(0, frameHeight)
        self.leftX.setRange(0, frameWidth)
        self.leftY.setRange(0, frameHeight)

        self.rightWidth.setRange(0, frameWidth)
        self.rightHeight.setRange(0, frameHeight)
        self.rightX.setRange(0, frameWidth)
        self.rightY.setRange(0, frameHeight)

        self.firstFrame = None
        while True:
            QtWidgets.QApplication.processEvents()

            line = None
            if(ser):
                line = ser.read(ser.in_waiting)

            if(line):
                self.status = line.decode().strip()

            if(self.status == 'go' and self.startTime == 0):
                self.startTime = int(time() * 1000)

            _, frame = cam.read()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)

            if self.firstFrame is None:
                self.firstFrame = gray
                continue

            frameDelta = cv2.absdiff(self.firstFrame, gray)
            thresh = cv2.threshold(frameDelta,  self.min.value(
            ), self.max.value(), cv2.THRESH_BINARY)[1]
            thresh = cv2.dilate(thresh, None, iterations=4)

            leftImage = thresh[self.leftY.value():self.leftY.value(
            ) + self.leftHeight.value(), self.leftX.value():self.leftX.value() + self.leftWidth.value()]

            rightImage = thresh[self.rightY.value():self.rightY.value(
            ) + self.rightHeight.value(), self.rightX.value():self.rightX.value() + self.rightWidth.value()]

            contoursLeft, _ = cv2.findContours(
                image=leftImage, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)

            contoursRight, _ = cv2.findContours(
                image=rightImage, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)

            if(len(contoursLeft) >= 1 and self.leftTime == 0):
                # Send left track timer stop
                if(ser):
                    ser.write(bytes('1', 'utf-8'))
                self.leftTime = int(time() * 1000) - self.startTime

            if(len(contoursRight) >= 1 and self.rightTime == 0):
                # Send right track timer stop
                if(ser):
                    ser.write(bytes('2', 'utf-8'))
                self.rightTime = int(time() * 1000) - self.startTime

            # Draw times
            if(self.leftTime != 0):
                cv2.putText(frame, str(self.leftTime), (self.leftX.value() + 10, self.leftY.value() + 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            if(self.rightTime != 0):
                cv2.putText(frame, str(self.rightTime), (self.rightX.value() + 10, self.rightY.value() + 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # Draw rectangles
            leftRectangle = cv2.rectangle(
                frame, (self.leftX.value(), self.leftY.value()), (self.leftX.value() + self.leftWidth.value(), self.leftY.value() + self.leftHeight.value()), (0, 0, 255), 2)

            rightRectangle = cv2.rectangle(
                leftRectangle, (self.rightX.value(), self.rightY.value()), (self.rightX.value() + self.rightWidth.value(), self.rightY.value() + self.rightHeight.value()), (0, 255, 0), 2)

            rgbFrame = None

            if(self.showThresh.checkState() == 2):
                rgbFrame = cv2.cvtColor(thresh, cv2.COLOR_GRAY2RGB)
                rgbFrame = cv2.resize(rgbFrame, (854, 480))
            else:
                rgbFrame = cv2.cvtColor(rightRectangle, cv2.COLOR_BGR2RGB)
                rgbFrame = cv2.resize(rgbFrame, (854, 480))

            qtImage = QtGui.QImage(
                rgbFrame, rgbFrame.shape[1], rgbFrame.shape[0], rgbFrame.strides[0], QtGui.QImage.Format_RGB888)
            self.label.setPixmap(QtGui.QPixmap.fromImage(qtImage))


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    MainWindow.setWindowTitle('BrickTimer')
    ui = Window()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
