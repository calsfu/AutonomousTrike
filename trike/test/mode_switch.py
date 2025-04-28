import Jetson.GPIO as GPIO
import time

def main():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(7, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(33, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(31, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    try:
        while True:
            # if not GPIO.input(11):
            #     print("Mode 1")
            # elif not GPIO.input(13):
            #     print("Mode 2")
            # elif not GPIO.input(15):
            #     print("Mode 3")
            print(GPIO.input(7), GPIO.input(33), GPIO.input(31))
            # time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()