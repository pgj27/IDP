import serial
import time
import struct

ser = serial.Serial('/dev/ttyACM0', 9600)
while True:
    line = ser.readline()
    response_rec = b'Got command\r\n'
    while(line != response_rec):
        print("got {}".format(line))
        ser.write(str.encode("cf"))
        ser.write(struct.pack("h", 100))
        line = ser.readline()
    print("Sent command successfully")

    response_fin = b'Finished moving\r\n'
    while(line != response_fin):
        print("got {}".format(line))
        time.sleep(0.1)
        line = ser.readline()
    print("Success")
