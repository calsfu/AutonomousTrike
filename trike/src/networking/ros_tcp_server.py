#!/usr/bin/env python
import rclpy
from std_msgs.msg import Char
from std_msgs.msg import Int8
from rclpy.node import Node
import socket

HOST = "127.0.0.1"
PORT = 65432
angles_dict = {
    0: "q", # leftmost
    1: "w",
    2: "e",
    3: "r",
    4: "t",
    5: "x", # straight
    6: "y",
    7: "u",
    8: "i",
    9: "o",
    10: "p" # rightmost
}

class Emergency_Brakes(Node):
    def __init__(self):
        super().__init__('autonomous_mode')
        self._on_off_ = self.create_subscription(
            Int8, '/mode', self.get_mode, 10) # 3 = autonomous
        self._brake = self.create_publisher(
            Int8, '/control/brake', 10) # 1 = brake
        self._steer = self.create_publisher(
            Char, '/control/steer', 10)
        
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))
            while True:
                s.listen()
                self.conn, addr = s.accept()
                with self.conn:
                    print(f"Connected by {addr}")
                    while True:
                        data = self.conn.recv(1024).decode()
                        if "s" in data:
                            steer = data.split("s")[1]
                            print(steer)
                            c = angles_dict[steer]
                            msg = Char()
                            msg.data = c
                            self._steer.publish(msg)
                        elif "b" in data:
                            brake = float(data.split("b")[1])
                            print(brake)
                            msg = Int8()
                            msg.data = brake
                            self._brake.publish(msg)
        
    def get_depth(self, msg: Int8):
        if msg.data == 3:
            self.conn.sendall("1".encode())
        else:
            self.conn.sendall("0".encode())

def main(args=None):
    rclpy.init(args=args)
    try:
        node = Emergency_Brakes()
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