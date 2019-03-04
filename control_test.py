import numpy as np
import time

pos = np.array([0.0, 0.0])
goal = np.array([100.0, 150.0])
angle = 0
error = goal - pos

while(np.linalg.norm(error) > 1):
    error = goal - pos
    error_dist = np.linalg.norm(error)
    error_angle = np.arctan2(error[1], error[0]) - angle
    speed = 0.1 * error_dist
    movement = np.array([speed * np.cos(angle), speed * np.sin(angle)])
    angle += 0.1 * error_angle
    pos += movement
    time.sleep(0.1)
    print("Error dist: {}, angle {}".format(error_dist, error_angle))
