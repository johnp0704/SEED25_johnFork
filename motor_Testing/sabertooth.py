# Niels Keller
# Sabertooth class
# 
# Relevant Documentation:
# https://www.dimensionengineering.com/datasheets/Sabertooth2x12.pdf


import numpy as np
import serial
import time


def linear_map_constrain_int(value, from_low, from_high, to_low, to_high):
        
        factor = (value - from_low)/(from_high - from_low) # return 0-1 float

        mapped_val = (to_high - to_low) * factor + to_low # Scale factor by range of output and add lower offset

        return round(min((to_high, max((to_low, mapped_val)) )) )


class SaberToothMotorDriver:
    
    def __init__(self, motor1_reversed, motor2_reversed):
        """
        Motor class constructor, takes roughly 1 sec to init. 
        
        Takes bool left_reversed, right_reversed to reverse the left and right directions  
        """

        self.sabertooth_UART_serial = serial.Serial( 
            port="/dev/ttyTHS0",
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            )

        #allow serial to startup
        time.sleep(1) #CHECK is this needed? last team had it

        self.motor1_reversed = motor1_reversed
        self.motor2_reversed = motor2_reversed

        self.__motor1_speed = 0
        self.__motor2_speed = 0


    def set_motor1_speed(self, speed):
        """
        Set motor speed

        Input is -100 to 100
        
        right = Motor 1 on bot
        """

        self.__motor1_speed = speed

        # From Datasheet: midpoint is stop
        range_start = 1
        range_max = 127
        range_midpoint = 64


        if (self.motor1_reversed):
            speed = -speed
        
        # default case is motor stop
        send_val = range_midpoint

        # Backwards
        if speed < 0:
            send_val = linear_map_constrain_int(100+speed, 0, 100, range_start, range_midpoint)

        #Forwards
        elif speed > 0:
            send_val = linear_map_constrain_int(speed, 0, 100, range_midpoint, range_max)

        self.sabertooth_UART_serial.write([send_val])


    def set_motor2_speed(self, speed):
        """
        Set motor speed

        Input is -100 to 100
        
        Left = Motor 2 on bot
        """

        self.__motor2_speed = speed

        # From Datasheet: midpoint is stop
        range_start = 128
        range_max = 255
        range_midpoint = 192


        if (self.motor2_reversed):
            speed = -speed
        
        # default case is motor stop
        send_val = range_midpoint

        # Backwards
        if speed < 0:
            send_val = linear_map_constrain_int(100+speed, 0, 100, range_start, range_midpoint)

        #Forwards
        elif speed > 0:
            send_val = linear_map_constrain_int(speed, 0, 100, range_midpoint, range_max)

        self.sabertooth_UART_serial.write([send_val])


    def get_motor1_speed(self):
        return self.__motor1_speed
    

    def get_motor2_speed(self):
        return self.__motor2_speed
    

    def all_motors_off(self):
        self.sabertooth_UART_serial.write([0])


    def updateMotorSpeed(self, left_speed, right_speed):
        """
        Set both speeds, range = -100 to 100
        
        """

        self.set_motor1_speed(right_speed)
        self.set_motor2_speed(left_speed)


    def __del__(self):

        #Send all stop when motor is killed
        self.sabertooth_UART_serial.write([0])







