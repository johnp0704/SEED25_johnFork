import sabertooth as st
import time


motor = st.SaberToothMotorDriver(True,True)

for i in range(0, -30, -1):
    print(i)
    motor.updateMotorSpeed(i,i)
    time.sleep(0.2)

for i in range(-30, 30):
    print(i)
    motor.updateMotorSpeed(i,i)
    time.sleep(0.2)

for i in range(30, 0):
    print(i)
    motor.updateMotorSpeed(i,i)
    time.sleep(0.2)

motor.all_motors_off()