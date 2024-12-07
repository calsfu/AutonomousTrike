#!/usr/bin/env python

import os
import sys
import csv
import numpy as np
import argparse
import logging
import random
#import pygame
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

try:
    sys.path.append('../PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg')
except IndexError:
    pass

import carla

saved_fRGB = []

class CarlaControlNode:
    def __init__(self):
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
        args = argparser.parse_args()
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)

        # Initialize ROS node
        rospy.init_node('carla_control_node', anonymous=True)
        self.host = rospy.get_param('~host', '127.0.0.1')
        self.port = rospy.get_param('~port', 2000)
        self.role_name = rospy.get_param('~role_name', 'self.vehicle')
        pub_driving = rospy.Publisher('driving_reporting', String, queue_size=10)
        pub_camera = rospy.Publisher('camera', String, queue_size=10)

        # Subscribe to cmd_vel
        #rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('driving_declaring', String, self.cmd_vel_callback)

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
        
        # OAK-D values
        camera_hz = .01
        camera_pixels_x = 320 #1280
        camera_pixels_y = 240 #800
        camera_horiz_fov = 72

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
            global saved_fRGB
            saved_fRGB.append(array)

        ego_front_rgb.listen(lambda image: save_fRGB(image))

        #self.vehicle.set_autopilot(True)

        # Do ROS stuff
        rate = rospy.Rate(10) # 10hz

        spectator = self.world.get_spectator()
        spectator.set_transform(self.vehicle.get_transform())
        world_snapshot = self.world.wait_for_tick() # make sure the camera has called back
        while not rospy.is_shutdown():
            spectator.set_transform(self.vehicle.get_transform())
            control = self.vehicle.get_control()
            rospy.loginfo("Throttle: " + str(control.throttle) + ", \tSteering: " + str((control.steer,0)[abs(control.steer) < 0.00001]) + ", \tBrake: " + str(control.brake))
            pub_driving.publish(str(control.throttle) + "," + str((control.steer,0)[abs(control.steer) < 0.00001]) + ",: " + str(control.brake))
            global saved_fRGB
            if len(saved_fRGB) != 0:
                csv_out = ','.join(['%f' % n for n in saved_fRGB[0].flatten()])
                pub_camera.publish(csv_out)
                saved_fRGB = [] # i dont like python its weird
            rate.sleep()

    # Drive the car based on ROS message
    def cmd_vel_callback(self, msg):
        # Convert Twist message to CARLA control
        # not actually using twist lol
        vals = msg.data.split(',')
        control = carla.VehicleControl()
        control.throttle = max(0.0, min(1.0, float(vals[0])))  # Forward/backward speed
        control.steer = max(-1.0, min(1.0, float(vals[1])))   # Steering
        self.vehicle.apply_control(control)

    #def run(self):
    #    rospy.spin()

if __name__ == '__main__':
    try:
        node = CarlaControlNode()
        #node.run()
    except rospy.ROSInterruptException:
        pass
