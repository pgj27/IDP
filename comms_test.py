import serial
import time
import struct
import numpy as np

ser = serial.Serial('/dev/ttyACM0', 9600)


def wait_for_message(msg):
    line = ser.readline()
    while(line != msg):
        print("heard: {}".format(str(line, 'utf-8', errors='ignore')))
        time.sleep(0.1)
        line = ser.readline()


while True:
    ready_for_path = b'Following path\r\n'
    distance_response = b'Distance received\r\n'
    finish_response = b'Finished moving\r\n'
    ready_for_steering = b'Constant movement\r\n'
    steering_response = b'Steering command\r\n'

    wait_for_message(ready_for_path)

    line = ser.readline()
    while(line != distance_response):
        ser.write(str.encode("cf"))
        ser.write(struct.pack("h", 100))
        print("Sent cf 100")
        print("heard: {}".format(str(line, 'utf-8', errors='ignore')))
        ser.write(str.encode("cf"))
        time.sleep(0.1)
        line = ser.readline()
    print("Sent command successfully")

    wait_for_message(ready_for_steering)
    print("Feedback")
    i = 0
    while(line != finish_response):
        steering = int(50 * np.cos(i * np.pi / 10))
        i += 1
        ser.write(str.encode("cs"))
        ser.write(struct.pack("h", steering))
        wait_for_message(steering_response)
        print("Sent cs {}".format(steering))
        print("heard: {}".format(str(line, 'utf-8', errors='ignore')))
        time.sleep(0.1)
        line = ser.readline()
    print("Success")
