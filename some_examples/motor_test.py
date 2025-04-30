import serial
import time

ser = serial.Serial("/dev/ttyUSB0",9600) 
forward_left = [74]
stop_left = [64]
forward_right = [202]
stop_right = [192]
 
ser.write(forward_left)
ser.write(forward_right)
time.sleep(2)
ser.write(stop_left)
ser.write(stop_right)

x = 1
while x < 100:
    left = [int(input('Set Left (1-127)'))]
    right = [int(input('Set Right (128-255)'))]
    print(type(left))
    print(type(right))
    ser.write(left)
    ser.write(right)
    time.sleep(1)
    x = x+1


print('done')
