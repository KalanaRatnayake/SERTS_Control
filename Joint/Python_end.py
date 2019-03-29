import serial
import struct

arduino = serial.Serial('/dev/ttyACM0', 9600)

while True:
    value = float(raw_input("Enter torque : "))
    print struct.pack('>f',value)
    arduino.write(struct.pack('>f',value))
