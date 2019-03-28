import serial
import struct
arduino = serial.Serial('/dev/tty.usbserial', 9600)

def binary(num):
    return (bin(ord(c)).replace('0b', '').rjust(8, '0') for c in struct.pack('!f', num))

while True:
    value = float(raw_input("Enter torque : "))
    byte = bin(value)
    
    
