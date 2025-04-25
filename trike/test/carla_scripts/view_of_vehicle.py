import os
import sys
import csv
import numpy as np
import argparse
import logging
import random
import pygame

try:
    sys.path.append('../PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg')
except IndexError:
    pass

import carla

class imu:
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None
    def set(self, xyz):
        self.x = xyz.x
        self.y = xyz.y
        self.z = xyz.z
    def get_x(self):
        return self.x
    def get_y(self):
        return self.y
    def get_z(self):
        return self.z

def main():
    imu_holder = imu()
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

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        ego_vehicle = None
        ego_front_rgb = None
        ego_back_rgb = None
        ego_front_depth = None
        ego_back_depth = None
        ego_col = None
        ego_lane = None
        ego_obs = None
        ego_gnss = None
        ego_imu = None

        camera_hz = .01
        camera_pixels_x = 320 #1280
        camera_pixels_y = 240 #800
        camera_horiz_fov = 72

        # --------------
        # Start recording
        # --------------
        data_path = 'data/'
        client.start_recorder(data_path + 'recording01.log') # dunno why this is here lol
        pygame.init()
        screen= pygame.display.set_mode((camera_pixels_x * 2, camera_pixels_y * 2))
        pygame.display.set_caption('image')
        surface = pygame.image.load("data/test.png").convert()
        screen.blit(surface, (0, 0))
        pygame.display.flip()
        
        # --------------
        # Spawn ego vehicle
        # --------------
        ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
        ego_bp.set_attribute('role_name','ego')
        print('\nEgo role_name is set')
        ego_color = random.choice(ego_bp.get_attribute('color').recommended_values)
        ego_bp.set_attribute('color',ego_color)
        print('\nEgo color is set')

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if 0 < number_of_spawn_points:
            random.shuffle(spawn_points)
            ego_transform = spawn_points[0]
            ego_vehicle = world.spawn_actor(ego_bp,ego_transform)
            print('\nEgo is spawned')
        else: 
            logging.warning('Could not found any spawn points')
        
        # --------------
        # Add the cameras to the ego vehicle. 
        # --------------
        
        # Front RGB
        cam_bp = None
        cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        cam_bp.set_attribute("image_size_x",str(camera_pixels_x))
        cam_bp.set_attribute("image_size_y",str(camera_pixels_y))
        cam_bp.set_attribute("fov",str(camera_horiz_fov))
        cam_bp.set_attribute("sensor_tick",str(camera_hz))
        cam_location = carla.Location(2,0,1)
        cam_rotation = carla.Rotation(0,0,0)
        cam_transform = carla.Transform(cam_location,cam_rotation)
        ego_front_rgb = world.spawn_actor(cam_bp,cam_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        # Back RGB
        cam_bp = None
        cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        cam_bp.set_attribute("image_size_x",str(camera_pixels_x))
        cam_bp.set_attribute("image_size_y",str(camera_pixels_y))
        cam_bp.set_attribute("fov",str(camera_horiz_fov))
        cam_bp.set_attribute("sensor_tick",str(camera_hz))
        cam_location = carla.Location(-3,0,1)
        cam_rotation = carla.Rotation(0,180,0)
        cam_transform = carla.Transform(cam_location,cam_rotation)
        ego_back_rgb = world.spawn_actor(cam_bp,cam_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        # Front Depth
        cam_bp = None
        cam_bp = world.get_blueprint_library().find('sensor.camera.depth')
        cam_bp.set_attribute("image_size_x",str(camera_pixels_x))
        cam_bp.set_attribute("image_size_y",str(camera_pixels_y))
        cam_bp.set_attribute("fov",str(camera_horiz_fov))
        cam_bp.set_attribute("sensor_tick",str(camera_hz))
        cam_location = carla.Location(2,0,1)
        cam_rotation = carla.Rotation(0,0,0)
        cam_transform = carla.Transform(cam_location,cam_rotation)
        ego_front_depth = world.spawn_actor(cam_bp,cam_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        # Back Depth
        cam_bp = None
        cam_bp = world.get_blueprint_library().find('sensor.camera.depth')
        cam_bp.set_attribute("image_size_x",str(camera_pixels_x))
        cam_bp.set_attribute("image_size_y",str(camera_pixels_y))
        cam_bp.set_attribute("fov",str(camera_horiz_fov))
        cam_bp.set_attribute("sensor_tick",str(camera_hz))
        cam_location = carla.Location(-3,0,1)
        cam_rotation = carla.Rotation(0,180,0)
        cam_transform = carla.Transform(cam_location,cam_rotation)
        ego_back_depth = world.spawn_actor(cam_bp,cam_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        # IMU
        cam_ld = None
        cam_ld = world.get_blueprint_library().find('sensor.other.imu')
        ld_location = carla.Location(0,0,0)
        ld_rotation = carla.Rotation(0,0,0)
        ld_transform = carla.Transform(ld_location,ld_rotation)
        ego_imu = world.spawn_actor(cam_ld,ld_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)

        def save_fRGB(image):
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            global saved_fRGB
            saved_fRGB = array

        def save_bRGB(image):
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            global saved_bRGB
            saved_bRGB = array

        def save_fDEPTH(image):
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            depth = np.empty([array.shape[0], array.shape[1], array.shape[2]])
            depth[:,:,0] = ((array[:,:,0] + array[:,:,1] * 256.0 + array[:,:,2] * 256.0 * 256.0)/((256.0 * 256.0 * 256.0) - 1))
            depth[:,:,1] = depth[:,:,0]
            depth[:,:,2] = depth[:,:,0]
            depth = depth * 1000
            global saved_fDEPTH
            saved_fDEPTH = depth

        def save_bDEPTH(image):
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            depth = np.empty([array.shape[0], array.shape[1], array.shape[2]])
            depth[:,:,0] = ((array[:,:,0] + array[:,:,1] * 256.0 + array[:,:,2] * 256.0 * 256.0)/((256.0 * 256.0 * 256.0) - 1))
            depth[:,:,1] = depth[:,:,0]
            depth[:,:,2] = depth[:,:,0]
            depth = depth * 1000
            global saved_bDEPTH
            saved_bDEPTH = depth

        def print_vals(image):
            imu_holder.set(image.accelerometer)
            control = ego_vehicle.get_control()
            print("Throttle: " + str(control.throttle) + ", \tSteering: " + str((control.steer,0)[abs(control.steer) < 0.00001]) + ", \tBrake: " + str(control.brake))
            print("X accel: " + str(imu_holder.get_x()) + ", \tY accel: " + str(imu_holder.get_y()) + ", \tZ accel: " + str(imu_holder.get_z()))
            print("")

        ego_front_rgb.listen(lambda image: save_fRGB(image))
        ego_back_rgb.listen(lambda image: save_bRGB(image))
        ego_front_depth.listen(lambda image: save_fDEPTH(image))
        ego_back_depth.listen(lambda image: save_bDEPTH(image))
        ego_imu.listen(lambda image: print_vals(image))

        #ego_front_rgb.listen(lambda image: image.save_to_disk(data_path + '%06d_fRGB.jpg' % image.frame) if image.frame % 10 == 0 else True)
        #ego_back_rgb.listen(lambda image: image.save_to_disk(data_path + '%06d_bRGB.jpg' % image.frame) if image.frame % 10 == 0 else True)
        #ego_front_depth.listen(lambda image: image.save_to_disk(data_path + '%06d_fDEPTH.jpg' % image.frame) if image.frame % 10 == 0 else True)
        #ego_back_depth.listen(lambda image: image.save_to_disk(data_path + '%06d_bDEPTH.jpg' % image.frame) if image.frame % 10 == 0 else True)

        
        # --------------
        # Place spectator on ego spawning
        # --------------
        spectator = world.get_spectator()
        world_snapshot = world.wait_for_tick() 
        spectator.set_transform(ego_vehicle.get_transform())
        
        # --------------
        # Enable autopilot for ego vehicle
        # --------------
        ego_vehicle.set_autopilot(True)
        
        # --------------
        # Game loop. Prevents the script from finishing.
        # --------------
        while True:
            world_snapshot = world.wait_for_tick()
            rgb = np.concatenate((saved_fRGB, saved_bRGB), axis=1)
            depth = np.concatenate((saved_fDEPTH, saved_bDEPTH), axis=1)
            img = np.concatenate((rgb, depth), axis=0)
            surface = pygame.surfarray.make_surface(img.swapaxes(0, 1))
            screen.blit(surface, (0, 0))
            pygame.display.flip()





    finally:
        pygame.quit()
        # --------------
        # Stop recording and destroy actors
        # --------------
        client.stop_recorder()
        if ego_vehicle is not None:
            if ego_front_rgb is not None:
                ego_front_rgb.stop()
                ego_front_rgb.destroy()
            if ego_back_rgb is not None:
                ego_back_rgb.stop()
                ego_back_rgb.destroy()
            if ego_front_depth is not None:
                ego_front_depth.stop()
                ego_front_depth.destroy()
            if ego_back_depth is not None:
                ego_back_depth.stop()
                ego_back_depth.destroy()
            if ego_col is not None:
                ego_col.stop()
                ego_col.destroy()
            if ego_lane is not None:
                ego_lane.stop()
                ego_lane.destroy()
            if ego_obs is not None:
                ego_obs.stop()
                ego_obs.destroy()
            if ego_gnss is not None:
                ego_gnss.stop()
                ego_gnss.destroy()
            if ego_imu is not None:
                ego_imu.stop()
                ego_imu.destroy()
            ego_vehicle.destroy()

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDone with tutorial_ego.')
