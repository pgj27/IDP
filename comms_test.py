import serial
import time

ser = serial.Serial('/dev/ttyACM1', 9600)
while True:
    ser.write(str.encode("cf"))
    ser.write(str.encode(chr(100)))
    ser.write(str.encode("e"))
    print(ser.readline())
    time.sleep(0.1)
