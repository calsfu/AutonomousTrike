import rospy
import pygame
import numpy as np
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

camera_pixels_x = 320 * 2 #1280
camera_pixels_y = 240 * 2 #800
bridge = CvBridge()

def driving_callback(data):
    rospy.loginfo("%s", data.data)

def showImage(img): # need this to be outside of the callback or things break because python is terrible
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    surface = pygame.surfarray.make_surface(img.swapaxes(0, 1))
    screen.blit(surface, (0, 0))
    pygame.display.flip()

def camera_callback(data):
    orig = bridge.imgmsg_to_cv2(data, "bgr8")
    drawImg = orig
    showImage(drawImg)
     
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("driving_reporting", String, driving_callback)
    rospy.Subscriber("camera", Image, camera_callback)
    rospy.spin()
    
if __name__ == '__main__':
    pygame.init()
    screen = pygame.display.set_mode((camera_pixels_x, camera_pixels_y))
    pygame.display.set_caption('image')
    surface = pygame.image.load("data/test.png").convert()
    screen.blit(surface, (0, 0))
    pygame.display.flip()

    listener()