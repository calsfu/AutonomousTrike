#RUN BEFORE:
#sudo apt-get update
#sudo apt-get install python3-pip
#sudo pip3 install Jetson.GPIO

import Jetson.GPIO as GPIO
import time
import keyboard

#pin definition
enA = 7 #PWM
in1 = 11
in2 = 9
EncoderA = 12
EncoderB = 13

position = 0

GPIO.setmode(GPIO.BOARD)

pwm = GPIO.PWM(enA, 1000)  # Set PWM frequency to 1kHz
pwm.start(0)  # Start with 0% duty cycle

# Setup GPIO pin as output
GPIO.setup(enA, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(in1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(enA, GPIO.OUT, initial=GPIO.LOW)

# Set up GPIO
GPIO.setup(EncoderA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(EncoderB, GPIO.IN, pull_up_down=GPIO.PUD_UP)


def encoder_callback(channel):
    global position
    if GPIO.input(EncoderA) == GPIO.input(EncoderB):
        position += 1  # Clockwise
    else:
        position -= 1  # Counterclockwise   

# Attach interrupt to count pulses
GPIO.add_event_detect(EncoderA, GPIO.BOTH, callback=encoder_callback)


def fmotor_control(speed):
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    pwm.ChangeDutyCycle(speed)  # Adjust speed (0-100%)

def stop_motor():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)

def fmotor_full():
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    pwm.ChangeDutyCycle(100)

def rmotor_control(speed):
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    pwm.ChangeDutyCycle(speed)  # Adjust speed (0-100%)

def rmotor_full():
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    pwm.ChangeDutyCycle(100)

def testReversible():
    #move motor
    fmotor_control(50)
    time.sleep(1)
    #stop
    stop_motor()
    time.sleep(.2)
    #move motor in opposite direction
    rmotor_control(50)
    time.sleep(1)
    #stop 
    stop_motor()
    time.sleep(2)

def testVariableSpeed():
    #move motor
    fmotor_control(10)
    print("10%")
    time.sleep(2)

    fmotor_control(25)
    print("25%")
    time.sleep(2)

    fmotor_control(50)
    print("50")
    time.sleep(2)

    fmotor_control(75)
    print("75")
    time.sleep(2)

    fmotor_control(100)
    print("100")
    time.sleep(2)
    
    stop_motor()

def testMaxSpeed():
    fmotor_full()
    time.sleep(2)
    stop_motor()

def testReverseSpeed():
    #move motor
    fmotor_control(10)
    print("10%")
    time.sleep(2)

    stop_motor()
    time.sleep(.2)

    rmotor_control(10)
    print("10%")
    time.sleep(2)

    ############

    fmotor_control(25)
    print("25%")
    time.sleep(2)

    stop_motor()
    time.sleep(.2)

    rmotor_control(25)
    print("25%")
    time.sleep(2)

    ############

    fmotor_control(50)
    print("50")
    time.sleep(2)
    
    stop_motor()
    time.sleep(.2)

    rmotor_control(50)
    print("50")
    time.sleep(2)

    ################

    fmotor_control(100)
    print("100")
    time.sleep(2)
    
    stop_motor()
    time.sleep(.2)

    rmotor_control(100)
    print("100")
    time.sleep(2)


def testEncoderFeedback():
    while position < 100:
        fmotor_control(20)
    rmotor_control(20)
    time.sleep(10)

#function Map
function_map = {
    '1':testReversible,
    '2':testVariableSpeed,
    '3':testMaxSpeed,
    '4':testReverseSpeed,
    '5':testEncoderFeedback
}

print("Press a number key (1-5) to execute a function. Press 'q' to quit.")


try:
    while True:
        key_event = keyboard.read_event()  # Read a key event
        
        if key_event.event_type == keyboard.KEY_DOWN:  # Detect key press
            key = key_event.name
            
            if key in function_map:  # Check if the key is mapped
                function_map[key]()  # Execute the mapped function
            
            elif key == 'q':  # Quit condition
                print("Exiting...")
                break

except KeyboardInterrupt:
    GPIO.cleanup()
    print("Program Exited")