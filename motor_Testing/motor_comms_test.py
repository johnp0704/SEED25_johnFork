
import numpy as np
import math
import serial
import time
import RPi.GPIO as GPIO

#time.sleep(20)

#for GPIO write to high or low
# pin = 12
# button_pin = 18



#pin setup
GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
# GPIO.setup(pin, GPIO.OUT)  #  pin set as output
# GPIO.output(pin, GPIO.HIGH) #start pin as high (arm is up)
# GPIO.setup(button_pin, GPIO.IN)  #  pin set as input
# prev_value = 1

#for motor controller serial usb
motor_driver = serial.Serial(
    port="/dev/ttyTHS0",
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
    #stop motors



# Wait a second to let the port initialize
time.sleep(1)



try:
    for i in range(50):

        print("high")
        motor_driver.write([70])   
        motor_driver.write([200])  

        time.sleep(1)
        
        print("low")
        motor_driver.write([0])
        time.sleep(1)

        while (motor_driver.inWaiting() > 0):

            data = motor_driver.read()
            print(data)




except KeyboardInterrupt:
    motor_driver.write([0])
    print("Exiting Program")

except Exception as exception_error:
    print("Error occurred. Exiting Program")
    print("Error: " + str(exception_error))

finally:
    motor_driver.close()
    pass







