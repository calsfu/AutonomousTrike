import sys
import select
import tty
import termios
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

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
        self.publisher_ = self.create_publisher(Int8, 'control/brake', 10)  # Create publisher for 'control/brake' topic
        self.timer = self.create_timer(
            1. / 20,  # Check keys at 50Hz
            self.timer_callback)

        self.getch = _Getch()

    def timer_callback(self):
        '''
        Timer function to be run to fetch a character if a key is pressed
        '''
        # key = self.getch.get_key()
        key = self.getch()
        msg = Int8()
        msg.data = 0
        if(key == 's'):
            msg.data = 1
        if(key == 'q'):
            self.get_logger().info(f'Shutdown initiated by {self.get_name()}')
            self.destroy_node()

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2
    node = KeyOp()  # Create node
    rclpy.spin(node)  # Keep the node running
    rclpy.shutdown()  # Shut down ROS2 when done

if __name__ == '__main__':
    main()
