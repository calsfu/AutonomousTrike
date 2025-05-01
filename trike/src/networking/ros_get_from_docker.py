#!/usr/bin/env python
import rclpy
from std_msgs.msg import Char
from std_msgs.msg import Int8
from rclpy.node import Node
import socket

HOST = "127.0.0.1"
PORT = 65433

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

class get_from_docker(Node):
    def __init__(self):
        super().__init__('get_from_docker')
        self._brake = self.create_publisher(
            Int8, '/control/brake', 10) # 1 = brake
        self._steer = self.create_publisher(
            Char, '/control/steer', 10)
        
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))
            s.listen()
            conn, addr = s.accept()
            with conn:
                # print(f"Connected by {addr}")
                while True:
                    data = conn.recv(1024).decode()
                    if not data:
                        continue
                    # print(data)
                    if "s" in data:
                        steer = int(data.split("s")[1])
                        c = angles_dict[steer]
                        # print("steer: " + c)
                        msg = Char()
                        msg.data = ord(c)
                        self._steer.publish(msg)
                    elif "b" in data:
                        brake = int(data.split("b")[1])
                        # print("brake: " + str(brake))
                        msg = Int8()
                        msg.data = brake
                        self._brake.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = get_from_docker()
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
