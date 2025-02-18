import Jetson.GPIO as GPIO
import time

# Pin definitions
enA = 32  # PWM pin
in1 = 31
in2 = 7
EncoderA = 15
EncoderB = 33

position = 0



def encoder_callback(channel):
    global position
    if GPIO.input(EncoderA) == GPIO.input(EncoderB):
        position += 1  # Clockwise
    else:
        position -= 1  # Counterclockwise   

# Attach interrupt to count pulses
# GPIO.add_event_detect(EncoderA, GPIO.BOTH, callback=encoder_callback)


def fmotor_control(speed):
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    pwm.ChangeDutyCycle(speed)  # Adjust speed (0-100%)

def stop_motor():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)

def fmotor_full():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    pwm.ChangeDutyCycle(100)

def rmotor_control(speed):
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    pwm.ChangeDutyCycle(speed)  # Adjust speed (0-100%)

def rmotor_full():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    pwm.ChangeDutyCycle(100)

def testReversible():
    print("Testing Reversible Motion")
    fmotor_control(50)
    time.sleep(1)
    stop_motor()
    time.sleep(0.2)
    rmotor_control(50)
    time.sleep(1)
    stop_motor()
    time.sleep(2)

def testVariableSpeed():
    print("Testing Variable Speed")
    for speed in [10, 25, 50, 75, 100]:
        fmotor_control(speed)
        print(f"{speed}% speed")
        time.sleep(2)
    stop_motor()

def testMaxSpeed():
    print("Testing Max Speed")
    fmotor_full()
    time.sleep(2)
    stop_motor()

def testReverseSpeed():
    print("Testing Reverse Speed")
    for speed in [10, 25, 50, 100]:
        fmotor_control(speed)
        print(f"Forward {speed}%")
        time.sleep(2)
        stop_motor()
        time.sleep(0.2)
        rmotor_control(speed)
        print(f"Reverse {speed}%")
        time.sleep(2)
        stop_motor()
        time.sleep(0.2)

def testEncoderFeedback():
    global position
    print("Testing Encoder Feedback")
    position = 0
    fmotor_control(20)
    while position < 100:
        time.sleep(0.1)
    stop_motor()
    rmotor_control(20)
    time.sleep(5)
    stop_motor()

def main():
    GPIO.setmode(GPIO.BOARD)

    # Setup GPIO pins
    GPIO.setup(enA, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(in1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(in2, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(EncoderA, GPIO.IN)
    GPIO.setup(EncoderB, GPIO.IN)

    # Set up PWM for motor speed control
    pwm = GPIO.PWM(enA, 50)  # Set PWM frequency to 1kHz
    pwm.start(25)  # Start with 0% duty cycle

    # Set up encoder pins
    # GPIO.setup(EncoderA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # GPIO.setup(EncoderB, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # Main execution sequence
    time.sleep(10)
    pwm.stop()
    # try:
    #     while True:
    #         testReversible()
    #         testVariableSpeed()
    #         testMaxSpeed()
    #         testReverseSpeed()
    #         # testEncoderFeedback()
        
    # except KeyboardInterrupt:
    #     print("Test interrupted")

    # finally:
    #     print("Cleaning up GPIO")
    #     pwm.stop()
    #     GPIO.cleanup()

if __name__ == 'main':
    main()