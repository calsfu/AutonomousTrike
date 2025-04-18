import rclpy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int8, Char
from rclpy.node import Node
import rclpy
import numpy as np
from cv_bridge import CvBridge

# tuning constants
noise_thresh = 1000
kblur = 20
safe_cm = 3000
boxes = 6
box_ring = 1 # can't be > boxes / 2
dimensions = [540, 960]

class Emergency_Brakes(Node):
    def __init__(self):
        super().__init__('emergency_brakes')
        self._sub_r_ = self.create_subscription(
            Image, '/oak/rgb/image_raw', self.get_rgb, 10)
        self._sub_d_ = self.create_subscription(
            Image, '/oak/stereo/image_raw', self.get_depth, 10)
        self._steer = self.create_publisher(
            Char, '/control/steer', 10)
        
    def get_rgb(self, msg: Image):
        # get image
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg)
        npimg = np.asarray(img)
        print("rgb" + str(npimg.shape))
        np.save('for_ssh/rgb', npimg)
        # self._brake.publish(brake)
        
    def get_depth(self, msg: Image):
        # get image
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg)
        npimg = np.asarray(img)
        print(npimg.shape)
        np.save('for_ssh/depth', npimg)
        # self._brake.publish(brake)

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
