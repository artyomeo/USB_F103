import serial
import time

ser = serial.Serial('COM6', baudrate=9600)

time.sleep(100)  # Wait 2 seconds

ser.close()