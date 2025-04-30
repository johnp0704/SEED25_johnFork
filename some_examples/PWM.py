import Jetson.GPIO as GPIO
import time


output_pin = 15
if output_pin is None:
    raise Exception('PWM not supported on this board')
def main():
    # Pin Setup:
    # Board pin-numbering scheme
    GPIO.setmode(GPIO.BOARD)
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)
    p = GPIO.PWM(output_pin, 500)
    val = 25
    incr = 5
    p.start(val)

    print("PWM running. Press CTRL+C to exit.")
    try:
        while True:
            time.sleep(0.25)
            if val >= 80:
                incr = -incr
            if val <= 0:
                incr = -incr
            val += incr
            p.ChangeDutyCycle(val)
            print(val)
    finally:
        p.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
