import Jetson.GPIO as GPIO
import time

# Pin definitions
enA = 32  # PWM pin
in1 = 31
in2 = 7
EncoderA = 15
EncoderB = 33

position = 0

# Set up GPIO mode
GPIO.setmode(GPIO.BOARD)

# Setup motor control pins
GPIO.setup(in1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(in2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(enA, GPIO.OUT)

# Initialize PWM on enA
pwm = GPIO.PWM(enA, 1000)  # Set frequency to 1kHz
pwm.start(0)  # Start with 0% duty cycle

# Set up Encoder pins (Remove pull-up setting)
GPIO.setup(EncoderA, GPIO.IN)
GPIO.setup(EncoderB, GPIO.IN)

# Encoder callback function
def encoder_callback(channel):
    global position
    A = GPIO.input(EncoderA)
    B = GPIO.input(EncoderB)
    
    if A == B:
        position += 1  # Clockwise
    else:
        position -= 1  # Counterclockwise

# Attach interrupt to count pulses
GPIO.add_event_detect(EncoderA, GPIO.BOTH, callback=encoder_callback)

# Motor control functions
def fmotor_control(speed):
    """Move forward at specified speed (0-100%)"""
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    pwm.ChangeDutyCycle(speed)

def stop_motor():
    """Stop motor"""
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    pwm.ChangeDutyCycle(0)

def rmotor_control(speed):
    """Move in reverse at specified speed (0-100%)"""
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    pwm.ChangeDutyCycle(speed)

def test_reversible():
    """Test bidirectional motion"""
    fmotor_control(50)
    time.sleep(1)
    stop_motor()
    time.sleep(0.2)
    rmotor_control(50)
    time.sleep(1)
    stop_motor()
    time.sleep(2)

def test_variable_speed():
    """Test increasing speed in steps"""
    for speed in [10, 25, 50, 75, 100]:
        fmotor_control(speed)
        print(f"Speed: {speed}%")
        time.sleep(2)
    stop_motor()

def test_encoder_feedback():
    """Test motor movement using encoder feedback"""
    global position
    position = 0  # Reset position
    fmotor_control(20)
    while position < 100:  # Move until 100 counts
        pass
    stop_motor()
    time.sleep(1)

# Cleanup function
def cleanup():
    print("Cleaning up GPIO...")
    GPIO.cleanup()

# Main function
if __name__ == "__main__":
    try:
        test_reversible()
        test_variable_speed()
        test_encoder_feedback()
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        cleanup()
