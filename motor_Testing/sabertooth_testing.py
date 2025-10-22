import sabertooth as st
import time


motor = st.SaberToothMotorDriver(True,True)

motor_test_range = 10
delay_time = 0.2

for i in range(0, -motor_test_range, -1):
    print(i)
    motor.updateMotorSpeed(i,i)
    time.sleep(delay_time)

for i in range(-motor_test_range, motor_test_range):
    print(i)
    motor.updateMotorSpeed(i,i)
    time.sleep(delay_time)

for i in range(motor_test_range, 0, -1):
    print(i)
    motor.updateMotorSpeed(i,i)
    time.sleep(delay_time)

motor.all_motors_off()