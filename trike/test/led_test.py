import Jetson.GPIO as GPIO
import time

pin = 33

GPIO.setmode(GPIO.BOARD)  # Important: BOARD mode means physical pin numbers
GPIO.setup(pin, GPIO.IN)

try:
    while True:
        value = GPIO.input(pin)
        print(f"Pin {pin}:", "HIGH" if value else "LOW")
        time.sleep(0.5)
except KeyboardInterrupt:
    GPIO.cleanup()
