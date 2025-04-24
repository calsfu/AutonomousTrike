import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

class BrakeCommand(Node):
    def __init__(self):
        super().__init__('brake_command')  # Node name
        self.publisher_ = self.create_publisher(Int8, 'control/brake', 10)  # Create publisher for 'control/brake' topic

        # Timer to call the callback function every 3 seconds
        self.timer = self.create_timer(3.0, self.publish_alternating_command)

        self.counter = 0  # Variable to alternate between 0 and 1

    def publish_alternating_command(self):
        msg = Int8()  # Create an Int8 message
        msg.data = self.counter % 2  # Alternate between 0 and 1
        self.publisher_.publish(msg)  # Publish the message

        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter += 1  # Increment counter to alternate the value

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2
    node = BrakeCommand()  # Create node
    rclpy.spin(node)  # Keep the node running
    rclpy.shutdown()  # Shut down ROS2 when done

if __name__ == '__main__':
    main()
