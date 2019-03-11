import numpy as np
import cv2
import serial
import time
import struct


class RobotControl:
    conts = []
    mask = []
    pos = (0, 0)
    start_pos = (0, 0)
    pos_buffer = []
    pos_buffer_location = 0
    angle = 0
    cells = []
    ser = serial.Serial('/dev/ttyACM0', 9600)
    reached_block = False

    def go_to_pos(self, goal):
        print("Pos: {}, Goal: {}".format(self.pos, goal))
        # Find angle to rotate and straight line distance
        angle_to_goal = np.arctan2(goal[1] - self.pos[1], goal[0] - self.pos[0]) - self.angle
        to_goal = (goal[0] - self.pos[0], goal[1] - self.pos[1])
        dist_to_goal = np.linalg.norm(to_goal)
        print("Rotate {}, forward {}".format(angle_to_goal, dist_to_goal))

        # Send commands to robot
        self.send_command("cr", int((angle_to_goal * 180 / np.pi)))
        self.send_command("cf", 6 * int(dist_to_goal))

    def deposit_cells(self):
        # Go to shelf
        shelf_pos = (self.start_pos[0], self.start_pos[1] + 240)
        self.go_to_pos(shelf_pos)

        # Rotate so back is facing shelf
        self.send_command("cr", self.angle % np.pi)

        # Deposit cells
        self.send_command("cd", 0)

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

    def wait_for_message(self, msg, timeout):
        waittime = 0.01
        i = 0
        i_max = timeout / waittime
        line = self.ser.readline()
        while line != msg:
            if i > i_max:
                print("Timed out waiting for {}".format(msg))
                return False
            if line == b'Finished with block\r\n':
                self.reached_block = True
            print("Waiting for {}, heard {}".format(msg, line))
            time.sleep(waittime)
            i += 1
            line = self.ser.readline()
        return True

    def send_command(self, command, param):
        ready = b'Following path\r\n'
        received = b'Command received\r\n'
        finish = b'Brake\r\n'

        if not self.wait_for_message(ready, 1000):
            return False

        line = self.ser.readline()
        self.ser.write(str.encode(command))
        self.ser.write(struct.pack("h", param))
        print("Sent {}: {}".format(command, param))
        while line != received:
            print("waiting for command received, heard: {}".format(str(line, 'utf-8', errors='ignore')))
            time.sleep(0.1)
            line = self.ser.readline()
        print("Sent command successfully")

        return True

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

        # Do first frame processing (find cells)
        if self.first_frame:
            self.first_frame = False
            self.conts, self.mask = self.process_first_frame(frameHSV)

        return ret, frame

    def __init__(self, cam):
        #self.cap = cv2.VideoCapture(cam)
        self.first_frame = True

        # Control
        self.send_command("cf", 2000)
        self.send_command("cr", 90)
        self.send_command("cf", 1000)
        self.send_command("cf", 300)
        self.send_command("cf", 300)
        self.send_command("cf", 300)
        self.send_command("cf", 300)
        self.send_command("cf", -500)
        self.send_command("cr", 90)
        self.send_command("cf", -800)
        self.send_command("cr", 188)
        self.send_command("cf", -1200)
        self.send_command("cu", 0)
        self.send_command("cr", -90)
        self.send_command("cf", 1200)
        self.send_command("cr", 90)

        while range(len(self.cells) > 0):
            closest = 0
            closest_dist = 10000
            for i in range(len(self.cells)):
                dist = np.linalg.norm((self.cells[i][0] - self.pos[0], self.cells[i][1] - self.pos[1]))
                if dist < closest_dist:
                    closest = i
                    closest_dist = dist
            self.go_to_pos(self.cells[closest])
            self.pos = self.cells[closest]
            np.delete(self.cells, closest)

        """while True:
            # Capture frame and process, break if no frame available
            if self.cap.isOpened():
                ret, frame = self.capture_frame()
                if ret == 0:
                    print("No frame")
                    break

            # Show robot position and heading (contours for corresponding points)
            cv2.drawContours(frame, self.conts, -1, (255, 0, 0), 3)
            cv2.circle(frame, self.pos, 20, (0, 255, 0), 3)
            arrow = (int(self.pos[0] + 50 * np.cos(self.angle)), int(self.pos[1] + 50 * np.sin(self.angle)))
            cv2.line(frame, self.pos, arrow, (0, 0, 255), 5)
            # Show filtered mask (for robot)
            cv2.imshow("mask", self.mask)
            # Display camera frame with overlay
            cv2.imshow("cam", frame)

            line = self.ser.readline()
            print(line)
            if line == b'Following path\r\n':
                self.plan_path()

            # Close program when q pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break"""


#rc = RobotControl("/home/philip/Videos/Webcam/robot-1.webm")
rc = RobotControl(1)
#rc.cap.release()
#cv2.destroyAllWindows()
