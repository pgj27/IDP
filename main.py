import numpy as np
import cv2
import serial
import time
import struct


class RobotControl:
    conts = []
    mask = []
    start_pos = (564, 447)
    pos = start_pos
    pos_buffer = []
    pos_buffer_location = 0
    angle = np.pi
    cells = []
    ser = serial.Serial('/dev/ttyACM0', 9600)

    def go_to_pos(self, goal):
        print("Pos: {}, Goal: {}".format(self.pos, goal))

        # Find angle to rotate and straight line distance
        to_goal = (goal[0] - self.pos[0], goal[1] - self.pos[1])
        angle_to_goal = np.arctan2(to_goal[1], to_goal[0]) - self.angle
        self.angle += angle_to_goal
        angle_to_goal *= 180 / np.pi  # Radians to degrees
        angle_to_goal %= 360  # All turns should be clockwise
        dist_to_goal = 4 * np.linalg.norm(to_goal)
        print("Rotate {}, forward {}".format(angle_to_goal, dist_to_goal))

        # Send commands to robot
        self.send_command("cr", int(angle_to_goal))
        self.send_command("cf", int(dist_to_goal))

    def deposit_cells(self):
        # Go to centre of table and rotate
        self.go_to_pos((320, 240))
        self.send_command("cr", int((np.pi - self.angle) * 180 / np.pi))

        # Reverse up to shelf
        self.send_command("cf", -1000)

        # Deposit cells
        self.send_command("cu", 0)

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
        # Loop until expected message arrived or time out
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

        # Wait until robot is ready for a path
        if not self.wait_for_message(ready, 1000):
            return False

        # Sent a movement command
        line = self.ser.readline()
        self.ser.write(str.encode(command))
        self.ser.write(struct.pack("h", param))
        print("Sent {}: {}".format(command, param))

        # Wait for command to be received
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
        #cv2.imshow('frame',frameHSV[:,:,0])
        #cv2.imshow('frame',frameHSV[:,:,1])
        #cv2.imshow('frame',frameHSV[:,:,2])

        # Do first frame processing (find cells)
        if self.first_frame:
            self.first_frame = False
            self.conts, self.mask = self.process_first_frame(frameHSV)
            # Show start location of robot and located cells
            cv2.drawContours(frame, self.conts, -1, (0, 0, 255), 3)
            cv2.circle(frame, self.pos, 10, (0, 255, 0), 3)
            cv2.imshow('frame',frame)
            #cv2.waitKey(0)

        return ret, frame

    def __init__(self, cam):

        # Collect/scan line of blocks at edge of table
        self.send_command("cf", 2000)  # Far edge
        self.send_command("cr", 80)  # Facing blocks, 80 deg so scrapes wall instead of passing blocks
        self.send_command("cf", 2000)
        self.send_command("cf", -800)
        self.send_command("cr", 90)
        self.send_command("cf", 600)
        self.send_command("cr", 180)
        self.send_command("cf", -1400)
        self.send_command("cu", 0)
        self.send_command("cr", 90)
        self.send_command("cf", 800)
        self.send_command("cr", -90)
        return

        """# Go to far corner so location is known
        self.send_command("cf", 500)
        self.pos = (50, 50)

        # Locate remaining celss
        self.cap = cv2.VideoCapture(cam)
        self.first_frame = True
        if self.cap.isOpened():
            ret, frame = self.capture_frame()
            if ret == 0:
                print("No frame")
                return

        # Go to each located cell in turn
        while range(len(self.cells) > 0):
            closest = 0
            closest_dist = 10000

            # Find closest cell
            for i in range(len(self.cells)):
                dist = np.linalg.norm((self.cells[i][0] - self.pos[0], self.cells[i][1] - self.pos[1]))
                if dist < closest_dist:
                    closest = i
                    closest_dist = dist
            self.go_to_pos(self.cells[closest])

            # Compensate for length of robot
            self.send_command("cf", 200)

            # Update robot position to position of collected cell
            self.pos = self.cells[closest]

            # Remove collected cell from list of cells to visit
            np.delete(self.cells, closest)

        # Route to shelf, rotate and unload conveyor
        self.deposit_cells()

        # Return to start position and orientation
        self.go_to_pos(self.start_pos)
        self.send_command("cr", int((np.pi - self.angle) * 180 / np.pi))"""


#rc = RobotControl("/home/philip/Videos/Webcam/robot-1.webm")
rc = RobotControl(1)
rc.cap.release()
cv2.destroyAllWindows()
