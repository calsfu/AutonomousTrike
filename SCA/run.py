from autonomous import run
import socket
from SCA_classes import Segmentation_Collision_Avoidance, Debug_Timer, Config

HOST = "127.0.0.1"
CLIENT_PORT = 65433
SHOW_WHAT_IT_SEES = False

sca = Segmentation_Collision_Avoidance("config", SHOW_WHAT_IT_SEES)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, CLIENT_PORT))

    while True:
        print("RUNNING AUNTONOMOUS")
        run(sca, s)
