#RUN BEFORE:
#sudo apt-get update
#sudo apt-get install python3-pip
#sudo pip3 install Jetson.GPIO

import Jetson.GPIO as GPIO
import time

#pin definition
enA = 9
in1 = 8
in2 = 7
EncoderA = 2
EncoderB = 3

GPIO.setmode(GPIO.BOARD)

# Setup GPIO pin as output
GPIO.setup(enA, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(in1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(enA, GPIO.OUT, initial=GPIO.LOW)

print("Press a number key (1-4) to execute a function. Press 'q' to quit.")


try:
    #test
    print("test")

except KeyboardInterrupt:
    GPIO.cleanup()
    print("Program Exited")