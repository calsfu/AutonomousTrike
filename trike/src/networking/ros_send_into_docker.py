#!/usr/bin/env python
import rclpy
from std_msgs.msg import Char
from std_msgs.msg import Int8
from rclpy.node import Node
import socket

HOST = "127.0.0.1"
PORT = 65432

class send_into_docker(Node):
    def __init__(self):
        super().__init__('send_into_docker')
        self._on_off_ = self.create_subscription(
            Int8, '/mode', self.get_mode, 10) # 3 = autonomous
        
    def get_mode(self, msg: Int8):
        while True:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect((HOST, PORT))
                    if msg.data == 3:
                        # print("turning on")
                        s.sendall("1".encode())
                    else:
                        # print("turning off")
                        s.sendall("0".encode())
                break
            except:
                print("message failed")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = send_into_docker()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()