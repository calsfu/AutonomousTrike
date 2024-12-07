import rospy
import pygame
import numpy as np
from std_msgs.msg import String

camera_pixels_x = 320 #1280
camera_pixels_y = 240 #800

def driving_callback(data):
    rospy.loginfo("%s", data.data)

def camera_callback(data):
    image = np.fromstring(data.data, sep=',')
    image = np.reshape(image, (camera_pixels_y, camera_pixels_x, 3))
    #print(image.shape)
    surface = pygame.surfarray.make_surface(image.swapaxes(0, 1))
    screen.blit(surface, (0, 0))
    pygame.display.flip()
     
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("driving_reporting", String, driving_callback)
    rospy.Subscriber("camera", String, camera_callback)
    rospy.spin()
    
if __name__ == '__main__':
    data_path = 'data/'
    pygame.init()
    screen = pygame.display.set_mode((camera_pixels_x, camera_pixels_y))
    pygame.display.set_caption('image')
    surface = pygame.image.load("data/test.png").convert()
    screen.blit(surface, (0, 0))
    pygame.display.flip()

    listener()