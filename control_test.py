import numpy as np
import time

pos = np.array([0.0, 0.0])
goal = np.array([100.0, 150.0])
angle = 0

#while (abs(pos[0] - goal[0]) > 1 or abs(pos[1] - goal[1]) > 1):
for i in range(20):
    error = goal - pos
    error_dist = np.linalg.norm(error)
    error_angle = np.arctan2(error[1], error[0]) - angle
    speed = 0.1 * error_dist
    movement = np.array([speed * np.cos(angle), speed * np.sin(angle)])
    angle += 0.2 * error_angle
    pos += movement
    time.sleep(0.1)
    print("Error dist: {}, angle {}".format(error_dist, error_angle))
