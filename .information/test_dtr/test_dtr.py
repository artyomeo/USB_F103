import serial
import time

ser = serial.Serial('COM6', baudrate=9600)
time.sleep(10)  # Wait 2 seconds

print(ser.rts)
print(ser.dtr)
ser.rts = False
# Set DTR
#ser.setDTR(True)
print(ser.rts)
print(ser.dtr)
ser.dtr = True
print(ser.rts)
print(ser.dtr)
time.sleep(5)  # Wait 2 seconds

# Clear DTR
ser.setDTR(False)
ser.setRTS(False)
print(ser.rts)
print(ser.dtr)

ser.close()
