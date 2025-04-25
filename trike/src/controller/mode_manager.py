import rclpy
import Jetson.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import Int16

PARK = 0
NEUTRAL = 1
MANUAL = 2
AUTONOMOUS = 3

class ModeManager(Node):
    def __init__(self):
        super().__init__('mode_manager')
        self.mode_publisher = self.create_publisher(Int16, 'mode', 10)
        self.mode = PARK
        self.neutral_pin = 7
        self.manual_pin = 33
        self.autonomous_pin = 31
        self.setup_gpio()
        self.create_timer(0.1, self.check_mode)
        self.mode_publisher.publish(Int16(data=self.mode))
        self.get_logger().info('Mode Manager Initialized')

    def setup_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(7, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(33, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(31, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def check_mode(self):
        if GPIO.input(self.neutral_pin) == GPIO.LOW:
            self.set_mode(NEUTRAL)
        elif GPIO.input(self.manual_pin) == GPIO.LOW:
            self.set_mode(MANUAL)
        elif GPIO.input(self.autonomous_pin) == GPIO.LOW:
            self.set_mode(AUTONOMOUS)
        else:
            self.set_mode(PARK)
        self.get_logger().info(f'Mode: {self.mode.name}')

    def set_mode(self, mode):
        if mode == self.mode: # Just in case
            return
        self.mode = mode
        msg = Int16()
        msg.data = self.mode
        self.mode_publisher.publish(msg)
        if self.mode == PARK:
            self.get_logger().info('Park Mode')
        elif self.mode == NEUTRAL:
            self.get_logger().info('Neutral Mode')
        elif self.mode == MANUAL:
            self.get_logger().info('Manual Mode')
        elif self.mode == AUTONOMOUS:
            self.get_logger().info('Autonomous Mode')
        
            

    