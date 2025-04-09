import rclpy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
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

class Emergency_Brakes(Node):
    def __init__(self):
        super().__init__('emergency_brakes')
        self._sub_ = self.create_subscription(
            Image, '/oak/stereo/image_raw', self.get_depth, 10)
        self._brake = self.create_publisher(
            Int8, '/control/brake', 10)
        
    def get_depth(self, msg: Image):
        # get image
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg)
        npimg = np.asarray(img)
        datatype = npimg.dtype
        dmax = np.iinfo(datatype).max
        # image processing
        edge = cv2.Sobel(src=npimg, ddepth=cv2.CV_16UC1, dx=1, dy=1, ksize=7)
        ksize = (kblur, kblur) 
        blur = cv2.blur(edge, ksize)  
        cleaned_img_zeros = np.where(blur < noise_thresh, npimg, 0)
        cleaned_img_np = np.where(cleaned_img_zeros == 0, np.nan, npimg)
        # finding nearest object
        x, y = cleaned_img_np.shape
        x, y = int(x / boxes), int(y / boxes)
        box_avgs = np.empty([boxes, boxes])
        for i in range(boxes):
            for j in range(boxes):
                box = cleaned_img_np[x * i:x * (i + 1),y * j:y * (j + 1)]
                box_avgs[i,j] = np.nanmean(box)
        box_avgs = box_avgs.astype(datatype)
        box_avgs = box_avgs[box_ring:-box_ring, box_ring:-box_ring]
        box_avgs = np.where(box_avgs == 0, dmax, box_avgs)
        # publishing value
        brake = Int8()
        if(np.min(box_avgs) < safe_cm):
            brake.data = 1
        else:
            brake.data = 0
        self._brake.publish(brake)

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
