import serial
import struct
import time as time

arduino = serial.Serial('/dev/ttyACM0', 9600)

while True:
    value = float(raw_input("Enter value : "))
    arduino.write(struct.pack('f',value))
    print struct.unpack("f", struct.pack('f',value))[0]
    time.sleep(1)
