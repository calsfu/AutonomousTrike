import sys
import numpy as np
import argparse
import logging
import random
import rospy
import pygame
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray

try:
    sys.path.append('../PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg')
except IndexError:
    pass
import carla

saved_fRGB = []
# OAK-D values
camera_hz = .01
camera_pixels_x = 320 * 2 #1280
camera_pixels_y = 240 * 2 #800
camera_horiz_fov = 72

class CarlaControlNode:
    def __init__(self):
        # Get argvs
        argparser = argparse.ArgumentParser(
            description=__doc__)
        argparser.add_argument(
            '--host',
            metavar='H',
            default='127.0.0.1',
            help='IP of the host server (default: 127.0.0.1)')
        argparser.add_argument(
            '-p', '--port',
            metavar='P',
            default=2000,
            type=int,
            help='TCP port to listen to (default: 2000)')
        argparser.add_argument(
            '-d', '--deploy',
            metavar='D',
            default=1,
            type=int,
            help='Take driving input from ViNT')
        args = argparser.parse_args()
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)

        # Initialize ROS node
        rospy.init_node('carla_control_node', anonymous=True)
        self.host = rospy.get_param('~host', '127.0.0.1')
        self.port = rospy.get_param('~port', 2000)
        self.role_name = rospy.get_param('~role_name', 'self.vehicle')
        pub_driving = rospy.Publisher('driving_reporting', String, queue_size=10)
        pub_camera = rospy.Publisher('camera', Image, queue_size=10)
        
        # Start up pygame to watch vehicle drive
        pygame.init()
        screen = pygame.display.set_mode((camera_pixels_x, camera_pixels_y))
        pygame.display.set_caption('image')
        surface = pygame.image.load("data/test.png").convert()
        screen.blit(surface, (0, 0))
        pygame.display.flip()

        # Connect to CARLA
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()

        # Spawn ego vehicle
        ego_bp = self.world.get_blueprint_library().find('vehicle.tesla.model3')
        ego_bp.set_attribute('role_name','ego')
        print('\nEgo role_name is set')
        ego_color = random.choice(ego_bp.get_attribute('color').recommended_values)
        ego_bp.set_attribute('color',ego_color)
        print('\nEgo color is set')

        spawn_points = self.world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if 0 < number_of_spawn_points:
            random.shuffle(spawn_points)
            ego_transform = spawn_points[0]
            self.vehicle = self.world.spawn_actor(ego_bp,ego_transform)
            print('\nEgo is spawned')
        else: 
            logging.warning('Could not found any spawn points')
        
        # Spawn camera
        cam_bp = None
        cam_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        cam_bp.set_attribute("image_size_x",str(camera_pixels_x))
        cam_bp.set_attribute("image_size_y",str(camera_pixels_y))
        cam_bp.set_attribute("fov",str(camera_horiz_fov))
        cam_bp.set_attribute("sensor_tick",str(camera_hz))
        cam_location = carla.Location(2,0,1)
        cam_rotation = carla.Rotation(0,0,0)
        cam_transform = carla.Transform(cam_location,cam_rotation)
        ego_front_rgb = self.world.spawn_actor(cam_bp,cam_transform,attach_to=self.vehicle, attachment_type=carla.AttachmentType.Rigid)
        
        def save_fRGB(image):
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            r,g,b = cv2.split(array)
            img = cv2.merge([b,g,r])
            global saved_fRGB
            saved_fRGB.append(img)

        ego_front_rgb.listen(lambda image: save_fRGB(image))

        # Watch the wheels, I guess
        spectator = self.world.get_spectator()
        spectator.set_transform(self.vehicle.get_transform())

        # For driving to create a rosbag, or using ViNT
        if args.deploy:
            rospy.Subscriber('waypoint', Float32MultiArray, self.waypoint)
        else:
            self.vehicle.set_autopilot(True)

        # Main loop
        rate = rospy.Rate(10) # 10hz
        bridge = CvBridge()
        world_snapshot = self.world.wait_for_tick() # make sure the camera has called back
        while not rospy.is_shutdown():
            spectator.set_transform(self.vehicle.get_transform())
            control = self.vehicle.get_control()
            pub_driving.publish(str(control.throttle) + "," + str((control.steer,0)[abs(control.steer) < 0.00001]) + ",: " + str(control.brake))
            global saved_fRGB
            if len(saved_fRGB) != 0:
                pub_camera.publish(bridge.cv2_to_imgmsg(saved_fRGB[0], "bgr8"))
                img = cv2.cvtColor(saved_fRGB[0], cv2.COLOR_BGR2RGB)
                surface = pygame.surfarray.make_surface(img.swapaxes(0, 1))
                screen.blit(surface, (0, 0))
                pygame.display.flip()
                saved_fRGB = []
            rate.sleep()

    def waypoint(self, msg):
        print(msg.data)
        control = carla.VehicleControl()
        control.throttle = max(0.0, min(1.0, float(msg.data[0])))  # Forward/backward speed
        control.steer = max(-1.0, min(1.0, float(msg.data[1])))   # Steering
        self.vehicle.apply_control(control)

if __name__ == '__main__':
    try:
        node = CarlaControlNode()
    except rospy.ROSInterruptException:
        pass
