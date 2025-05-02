#!/usr/bin/env python
import rclpy
import Jetson.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import Int8, Char

PARK = 0
NEUTRAL = 1
MANUAL = 2
AUTONOMOUS = 3

# TODO: Put audio comands in a shared file
AUTONOMOUS_ON = 0
BRAKES_OFF = 1
BRAKES_ON = 2
DESTINATION_CONFIRMED = 3
DESTINATION_SET = 4
MANUAL_ON = 5
NEUTRAL_ON = 6
PARK_ON = 7
NO_GPS_SIGNAL = 8
ENTER_DESTINATION = 9
SYSTEM_READY = 10
TURNING_LEFT = 11
TURNING_RIGHT = 12
CONFIRM_DESTINATION = 13

class ModeManager(Node):
    def __init__(self):
        super().__init__('mode_manager')
        self.mode_publisher = self.create_publisher(Int8, 'mode', 10)
        self.audio_publisher = self.create_publisher(Int8, 'audio_command', 10)
        self.steering_publisher = self.create_publisher(Char, '/control/steering/new', 10)

        self.int_to_mode = {
            PARK: 'Park',
            NEUTRAL: 'Neutral',
            MANUAL: 'Manual',
            AUTONOMOUS: 'Autonomous'
        }
        self.int_to_audio = {
            PARK: PARK_ON,
            NEUTRAL: NEUTRAL_ON,
            MANUAL: MANUAL_ON,
            AUTONOMOUS: AUTONOMOUS_ON
        }
        self.mode = PARK
        self.neutral_pin = 7
        self.manual_pin = 33
        self.autonomous_pin = 31
        self.setup_gpio()
        self.create_timer(0.1, self.check_mode)
        self.mode_publisher.publish(Int8(data=self.mode))
        self.get_logger().info('Mode Manager Initialized')

    def setup_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(7, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(33, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(31, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def check_mode(self):
        if GPIO.input(self.neutral_pin) == GPIO.LOW:
            self.set_mode(MANUAL)
        elif GPIO.input(self.manual_pin) == GPIO.LOW:
            self.set_mode(AUTONOMOUS)
        # elif GPIO.input(self.autonomous_pin) == GPIO.LOW:
        #     self.set_mode(AUTONOMOUS)
        else:
            self.set_mode(PARK)
        # self.get_logger().info(f'Mode: {self.mode}')

    def set_mode(self, mode):
        if mode == self.mode: # Just in case
            return
            
        self.mode = mode
        msg = Int8()
        msg.data = self.mode
        self.mode_publisher.publish(msg)
        self.get_logger().info(f'{self.int_to_mode[self.mode]}')
        audio_msg = Int8()
        audio_msg.data = self.int_to_audio[self.mode]
        self.audio_publisher.publish(audio_msg)
        
            
def main(args=None):
    rclpy.init(args=args)
    mode_manager = ModeManager()
    rclpy.spin(mode_manager)
    mode_manager.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()