import numpy as np
import cv2
import serial
import time
import struct


class RobotControl:
    conts = []
    mask = []
    pos = (0, 0)
    pos_buffer = []
    pos_buffer_location = 0
    angle = 0
    cells = []
    #ser = serial.Serial('/dev/ttyACM0', 9600)

    def find_line(self, conts):
        min_x = 1000
        max_x = 0
        line_y = []
        line_y_right = []
    
        # Find furthest left and right points
        for i in range(len(conts)):
            if conts[i][0][0][0] < min_x:
                min_x = conts[i][0][0][0]
            if conts[i][0][0][0] > max_x:
                max_x = conts[i][0][0][0]
    
        # Find points in a line with these
        for i in range(len(conts)):
            if conts[i][0][0][0] - min_x < 10:
                line_y.append(conts[i][0][0][1])
            if conts[i][0][0][0] - max_x > -10:
                line_y_right.append(conts[i][0][0][0])
    
        # Choose the longer line as the reference point
        if len(line_y_right) > len(line_y):
            line_y = line_y_right
    
        # Average with distance to find scale
        sum = 0
        if len(line_y) < 2:
            print("Couldn't find line of cells, no scale could be found")
            return 5
        for i in range(len(line_y)-1):
            sum += line_y[i] - line_y[i+1]
        scale = sum / (15 * (len(line_y)-1))
        print("scale is {} pixels/cm".format(scale))
        return scale

    def process_first_frame(self, img):
        # Set parameters to filter
        lowerBound = np.array([80, 30, 200])
        upperBound = np.array([120, 200, 255])
        kernelOpen = np.ones((2, 2))
        kernelClose = np.ones((10, 10))
        lengthUpper = 30
        lengthLower = 10
    
        # Filter points
        mask = cv2.inRange(img, lowerBound, upperBound)
        maskOpen = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernelOpen)
        maskClose = cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE, kernelClose)
        maskFinal = maskClose
        conts, h = cv2.findContours(maskFinal.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        to_remove = []
        for i in range(len(conts)):
            if len(conts[i]) < lengthLower:
                to_remove.append(i)
        conts = np.delete(conts, to_remove, 0)

        # Store cell locations
        print("Found {} cells:".format(len(conts)))
        for c in conts:
            self.cells.append(c[0][0])
            print(c[0][0])
    
        return conts, maskFinal

    def find_robot(self, img):
        # Set parameters to filter for robot
        lowerBoundMarking = np.array([0, 50, 200])
        upperBoundMarking = np.array([15, 255, 255])
        lowerBoundBody = np.array([10, 0, 230])
        upperBoundBody = np.array([50, 30, 255])
        kernelOpen = np.ones((2, 2))
        kernelCloseBody = np.ones((15, 15))
        kernelCloseMarking = np.ones((5, 5))
        lengthLower = 30

        # Filter points
        maskBody = cv2.inRange(img, lowerBoundBody, upperBoundBody)
        maskBodyOpen = cv2.morphologyEx(maskBody, cv2.MORPH_OPEN, kernelOpen)
        maskBodyClose = cv2.morphologyEx(maskBodyOpen, cv2.MORPH_CLOSE, kernelCloseBody)
        maskMarking = cv2.inRange(img, lowerBoundMarking, upperBoundMarking)
        maskMarkingClose = cv2.morphologyEx(maskMarking, cv2.MORPH_CLOSE, kernelCloseMarking)
        maskFinal = np.uint8((0.5 * maskMarkingClose + 0.5 * maskBodyClose) / 255)
        conts, h = cv2.findContours(maskFinal.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        to_remove = []
        for i in range(len(conts)):
            dist_to_pos = np.linalg.norm(conts[i][int(len(conts[i])/2)][0] - self.pos)
            if len(conts[i]) < lengthLower or (dist_to_pos > 50 and self.pos != (0, 0)):
                to_remove.append(i)
        conts = np.delete(conts, to_remove, 0)

        # Get robot orientation

        # If robot is found, update position to average point location
        if len(conts) > 0:
            x_sum = 0
            y_sum = 0
            angle_sum = 0
            n = 0
            for c in conts:
                p, m, angle = cv2.fitEllipse(c)
                angle_sum += angle * np.pi / 180
                x_sum += c[int(len(c)/2)][0][0]
                y_sum += c[int(len(c)/2)][0][1]
                n += 1

            # Average angles to get robot heading
            self.angle = angle_sum / n

            # Update position smoothing buffer
            if len(self.pos_buffer) < 5:
                self.pos_buffer.append((int(x_sum / n), int(y_sum / n)))
            else:
                self.pos_buffer[self.pos_buffer_location] = (int(x_sum / n), int(y_sum / n))
            self.pos_buffer_location = (self.pos_buffer_location + 1) % 5
            print("pos: {}".format(self.pos))

        # Smooth robot position
        x_sum = 0
        y_sum = 0
        n = 0
        for i in self.pos_buffer:
            x_sum += i[0]
            y_sum += i[1]
            n += 1
        self.pos = (int(x_sum / n), int(y_sum / n))

        return conts, 255 * maskFinal

    def wait_for_message(self, msg):
        line = self.ser.readline()
        while line != msg:
            print("Waiting for {}, heard {}".format(msg, line))
            time.sleep(0.01)
            line = self.ser.readline()

    def send_command(self, command, param):
        ready = b'Following path\r\n'
        received = b'Command received\r\n'
        finish = b'Finished moving\r\n'

        self.wait_for_message(ready)

        line = self.ser.readline()
        while line != received:
            self.ser.write(str.encode(command))
            self.ser.write(struct.pack("h", param))
            print("Sent {}: {}".format(command, param))
            print("heard: {}".format(str(line, 'utf-8', errors='ignore')))
            time.sleep(0.1)
            line = self.ser.readline()
        print("Sent command successfully")

        self.wait_for_message(finish)
        print("Executed command successfully")
        return 0

    def capture_frame(self):
        # Capture frame-by-frame
        ret, frame = self.cap.read()
        if ret == 0:
            return ret
    
        # Convert frame to HSV (imshow used for calibration)
        frame = cv2.resize(frame, (640, 480))
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #cv2.imshow('h',frameHSV[:,:,0])
        #cv2.imshow('s',frameHSV[:,:,1])
        #cv2.imshow('v',frameHSV[:,:,2])

        # Do first frame processing (find cells, scale)
        if self.first_frame:
            self.first_frame = False
            self.conts, self.mask = self.process_first_frame(frameHSV)

        # Find robot
        self.conts, self.mask = self.find_robot(frameHSV)
    
        # Show robot position and heading (contours for corresponding points)
        cv2.drawContours(frame, self.conts, -1, (255, 0, 0), 3)
        cv2.circle(frame, self.pos, 20, (0, 255, 0), 3)
        arrow = (int(self.pos[0] + 50 * np.cos(self.angle)), int(self.pos[1] + 50 * np.sin(self.angle)))
        cv2.line(frame, self.pos, arrow, (0, 0, 255), 5)
        # Show filtered mask (for robot)
        cv2.imshow("mask", self.mask)
        # Display camera frame with overlay
        cv2.imshow("cam", frame)

        return ret

    def __init__(self, cam):
        self.cap = cv2.VideoCapture(cam)
        self.first_frame = True
        while True:
            # Capture frame and process, break if no frame available
            if self.cap.isOpened():
                ret = self.capture_frame()
                if ret == 0:
                    break

            # Control

            # Close program when q pressed
            if cv2.waitKey(0) & 0xFF == ord('q'):
                break


rc = RobotControl("/home/philip/Videos/Webcam/robot-1.webm")
#rc = RobotControl(1)
rc.cap.release()
cv2.destroyAllWindows()
