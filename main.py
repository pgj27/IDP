import numpy as np
import matplotlib.pyplot as plt
import cv2


class RobotControl:
    conts = []
    mask = []

    def plan_path(self, grid):
        to_visit = [[0,0]]
        num_nodes = 1
        for i in range(len(grid)):
            for j in range(len(grid[i])):
                if grid[i][j] == 1:
                    to_visit.append([i,j])
                    num_nodes += 1
    
        weights = np.zeros((num_nodes, num_nodes), dtype=int)
        for i in range(num_nodes):
            for j in range(num_nodes):
                weights[i][j] = abs(to_visit[i][0] - to_visit[j][0]) + abs(to_visit[i][1] - to_visit[j][1])
                if i == j:
                    weights[i][j] = 1000
    
        route = [[0, 0]]
        pos = 0
        for i in range(num_nodes-1):
            next = np.argmin(weights[pos])
            print("Closest point to {} is {} (distance: {})".format(pos, next, weights[pos, next]))
            xstart = to_visit[pos][1]
            ystart = to_visit[pos][0]
            xend = to_visit[next][1]
            yend = to_visit[next][0]
            if xstart > xend:
                xend = xstart+1
                xstart = to_visit[next][1]+1
            if ystart > yend:
                yend = ystart+1
                ystart = to_visit[next][0]+1
            print("{}, {} to {}, {}".format(xstart, ystart, xend, yend))
            grid[ystart:yend, to_visit[pos][1]] += 0.1
            grid[to_visit[next][0], xstart:xend] += 0.1
            weights[:, pos] = 1000
            pos = next
            route.append(to_visit[pos])
        route.append([0, 0])
        print(route)
    
        plt.matshow(grid)
        plt.show()

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
        #print("scale is {} pixels/cm".format(scale))
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
    
        # Generate grid to store cell locations
        scale = self.find_line(conts)
        grid_scale = 3 * scale
        grid = np.zeros(((int(img.shape[0] / grid_scale)), int(img.shape[1] / grid_scale)))
        print(grid.shape)
        #print("Cells found at: ")
        for i in range(len(conts)):
            xi = int(conts[i][0][0][0] / grid_scale + 1)
            yi = int(conts[i][0][0][1] / grid_scale + 1)
            #print("{}, {}".format(xi, yi))
            grid[yi, xi] = 1 # numpy arrays are row-col
    
        self.plan_path(grid)
    
        return conts, maskFinal

    def find_robot(self, img):
        return 0

    def send_command(self, command):
        # send a command over serial
        # wait for a response
        # find robot
        # check position is as expected
        # if not re-plan route
        return 0

    def capture_frame(self):
        # Capture frame-by-frame
        ret, frame = self.cap.read()
        if ret == 0:
            return ret
    
        # Our operations on the frame come here
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
        # Display the resulting frame
        cv2.imshow('h',frameHSV[:,:,0])
        cv2.imshow('s',frameHSV[:,:,1])
        cv2.imshow('v',frameHSV[:,:,2])
    
        # Do first frame processing
        if self.first_frame:
            self.first_frame = False
            self.conts, self.mask = self.process_first_frame(frameHSV)
    
        # Show locations of cells
        cv2.drawContours(frame, self.conts, -1, (255, 0, 0), 3)
        cv2.imshow("mask", self.mask)
        cv2.imshow("cam", frame)
        return ret

    def __init__(self, cam):
        self.cap = cv2.VideoCapture(cam)
        self.first_frame = True
        while(True):
            if self.cap.isOpened():
                ret = self.capture_frame()
                if ret == 0:
                    break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


rc = RobotControl("/home/philip/Videos/Webcam/table-3_2.webm")
#rc = RobotControl(0)
rc.cap.release()
cv2.destroyAllWindows()
