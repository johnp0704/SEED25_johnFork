import RPi.GPIO as GPIO
import time

# Pin Definitons:
led_pin = 12  # Board pin 12
but_pin = 18  # Board pin 18

def main():
    # Pin Setup:
    GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
    GPIO.setup(led_pin, GPIO.OUT)  # LED pin set as output
    GPIO.setup(but_pin, GPIO.IN)  # button pin set as input
    prev_value = 0
    # Initial state for LEDs:
    GPIO.output(led_pin, GPIO.LOW)

    print("Starting demo now! Press CTRL+C to exit")
    try:
        while True:
            print("Waiting for button event")
            current_value = GPIO.input(but_pin)
            print(f"current_value: {current_value}")
            print(f"prev_value: {prev_value}")
            if(current_value != prev_value):
                # event received when button pressed
                print("Button Pressed!")
                GPIO.output(led_pin, GPIO.HIGH)
                time.sleep(1)
                print("changing output to low.")
                GPIO.output(led_pin, GPIO.LOW)

    finally:
        GPIO.cleanup()  # cleanup all GPIOs

if __name__ == '__main__':
    main()
