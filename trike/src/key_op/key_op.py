#!/usr/bin/env python
import sys
import select
import tty
import termios
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Float32, Char

class _Getch(object):
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self):
        return self.impl()


class _GetchUnix(object):
    def __init__(self):
        import sys
        import tty

    def __call__(self):
        import sys
        import termios
        import tty
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows(object):
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()

class KeyOp(Node):
    '''
    Allows for keyboard operation of brakes.
    TODO: Add another publisher for steering control
    '''
    def __init__(self):
        '''
        Creates a publisher, timer, and getch object
        '''
        super().__init__('key_op')  # Node name
        self.brake_publisher_ = self.create_publisher(Int8, 'control/brake', 10)  # publisher for braking
        self.steer_publisher_ = self.create_publisher(Char, 'control/steer', 10) # publisher for steering
        self.timer_ = self.create_timer(
            1. / 20,  # Check keys at 50Hz
            self.timer_callback)

        self.getch_ = _Getch()
        self.last_steer = 0
        self.last_brake = 0

    def timer_callback(self):
        '''
        Timer function to be run to fetch a character if a key is pressed
        '''
        key = self.getch_()

        # check for quit
        if(key == 'q'):
            self.get_logger().info(f'Shutdown initiated by {self.get_name()}')
            self.destroy_node()
        # check for activate brake
        # 0 for activate brake, 1 for release brake

        if self.is_brake_command(key):
            # w for activate brake, s for release brake
            brake_msg = Int8()
            brake_msg.data = self.last_brake
            if key == 'w':
                brake_msg.data = 0
            elif key == 's':
                brake_msg.data = 1
            self.brake_publisher_.publish(brake_msg)
            self.last_brake = brake_msg.data       
            self.get_logger().info(f'Publishing: {brake_msg.data} to /control/brake')
            
        elif self.is_steer_command(key):
            # j for left, k for straight, l for right
            steer_msg = Char()
            steer_msg.data = self.last_steer
            steer_msg.data = ord(key)
            self.steer_publisher_.publish(steer_msg)
            self.last_steer = steer_msg.data
            self.get_logger().info(f'Publishing: {steer_msg.data} to /control/steer')
        
    def is_brake_command(self, key):
        '''
        Checks if the key pressed is a brake command
        '''
        return key == 'w' or key == 's'
    
    def is_steer_command(self, key):
        '''
        Checks if the key pressed is a steer command
        '''
        return key == 'q' or key == 'w' or key == 'e' or key == 'r' or key == 't' or key == 'y' or key == 'u' or key == 'i' or key == 'o' or key == 'p' or key == 'j' or key == 'k' or key == 'l' or key == 'd'

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2
    node = KeyOp()  # Create node
    rclpy.spin(node)  # Keep the node running
    rclpy.shutdown()  # Shut down ROS2 when done

if __name__ == '__main__':
    main()
