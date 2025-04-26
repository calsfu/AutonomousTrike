#!/usr/bin/env python
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64

import controller as con
import robot_model as rm

class ControllerLine(Node):
    def __init__(self):
        '''
        Create Controller Line
        
        Whoever tunes this look up how a PID controller works. You will need to change the gain values.
        '''
        super().__init__('controller_line')
        self.lin_speed = 0.5
        self.gain_kp = 5e-3
        self.gain_kd = 1e-3
        self.gain_ki = 0.0
        self.subscriber = self.create_subscription(PointStamped, '/image/centroid', self.sub_callback, 1)
        self.twist_publisher = self.create_publisher(Twist, 'robot_twist', 10)
        self.control_publisher = self.create_publisher(Float64, 'control_error', 10)
        self.pid = con.PID(self.gain_kp, self.gain_kd, self.gain_ki)
        self.stamped_msg_register = rm.StampedMsgRegister()
        self.image_width = 1920

    def sub_callback(self, msg:PointStamped):
        '''
        Callback to subscriber
        Computes error signal and time_delay and publishes to 
        two publishers
        '''

        error_signal = Float64()
        error_signal.data = msg.point.x - self.image_width / 2
        self.control_publisher.publish(error_signal)
        error_signal = float(msg.point.x - self.image_width / 2)
        time_delay, _ = self.stamped_msg_register.replace_and_compute_delay(msg)
        if time_delay is None:
            return
        twist_msg = Twist()
        twist_msg.linear.x = self.lin_speed # we don't care about the linear speed since we can't control it
        twist_msg.angular.z = self.pid.proportional(error_signal) + self.pid.derivative(error_signal, time_delay) + self.pid.integral(error_signal, time_delay)
        self.twist_publisher.publish(twist_msg)

def main(args=None):
    '''
    Node setup and ROS loop
    '''
    rclpy.init(args=args)
    controller_line = ControllerLine()
    rclpy.spin(controller_line)
    controller_line.destroy_node()
    rclpy.shutdown()