import serial
import time
import struct

ser = serial.Serial('/dev/ttyACM0', 9600)
while True:
    ser.write(str.encode("cf"))
    ser.write(struct.pack("h", 2000))
    ser.write(str.encode("e"))
    print(ser.readline())
    time.sleep(0.1)