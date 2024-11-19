import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from fastsam import FastSAM, FastSAMPrompt
from cv_bridge import CvBridge

class FastSam(Node):
    def __init__(self):
        super().__init__('fastsam')
        self.subscription = self.create_subscription(
            Image,
            '/D415_1/color/image_raw',  # Replace with the actual topic name
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(
            Image,
            '/fastsam/image',
            10
        )

        self.bridge = CvBridge()

        self.device = 'cpu'


        #fast sam
        model = FastSAM('./weights/FastSAM-s.pt')

    def image_callback(self, msg):  
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        everything_results = self.model(cv_image, device=self.DEVICE, retina_masks=True, imgsz=1024, conf=0.4, iou=0.9,)
        prompt_process = FastSAMPrompt(cv_image, everything_results, device=self.DEVICE)

        # everything prompt
        ann = prompt_process.everything_prompt()

        # prompt_process.plot(annotations=ann,output_path='./test_img.jpg',)

        processed_image = self.bridge.cv2_to_imgmsg(ann, encoding='bgr8')

        self.publisher.publish(processed_image) 



def main(args=None):
    rclpy.init(args=args)
    node = FastSam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
