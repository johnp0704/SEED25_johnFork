import sabertooth as st
import time


motor = st.SaberToothMotorDriver(True,True)

for i in range(-30,30):
    motor.updateMotorSpeed(i,i)
    time.sleep(0.2)

motor.all_motors_off()