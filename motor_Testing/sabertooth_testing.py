import sabertooth as st
import time


motor = st.SaberToothMotorDriver(True,True)

for i in range(0, -30):
    motor.updateMotorSpeed(i,i)
    time.sleep(0.2)

for i in range(-30, 30):
    motor.updateMotorSpeed(i,i)
    time.sleep(0.2)

for i in range(30, 0):
    motor.updateMotorSpeed(i,i)
    time.sleep(0.2)

motor.all_motors_off()