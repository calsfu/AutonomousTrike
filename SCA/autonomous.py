import cv2
import numpy as np
import argparse
import os
from time import sleep
import datetime
from PIL import Image
from SCA_classes import Segmentation_Collision_Avoidance, Debug_Timer, Config
import matplotlib.pyplot as plt
import multiprocessing
from multiprocessing import shared_memory
import threading
import socket
import struct
import select
from timeit import default_timer as timer
from emergency_brake import Emergency_Brake

SHOW_WHAT_IT_SEES = False

HOST = "127.0.0.1"
SERVING_PORT = 65432
CLIENT_PORT = 65433

last_rectif_right = None
right_intrinsic = [[860.0, 0.0, 640.0],
                   [0.0, 860.0, 360.0],
                   [0.0, 0.0, 1.0]]
lrcheck   = True      # Better handling for occlusions
extended  = False     # Closer-in minimum depth, disparity range is doubled
subpixel  = False     # Better accuracy for longer distance, fractional disparity
pcl_converter = None  # (Set this if you have a point cloud visualizer)

Config.load("config")

noise_thresh = Config.get("noise_thresh")
kblur = Config.get("kblur")
safe_cm = Config.get("safe_cm")
print(safe_cm)
boxes = Config.get("boxes")
box_ring = Config.get("box_ring")
ebrake = Emergency_Brake(noise_thresh, kblur, safe_cm, boxes, box_ring, SHOW_WHAT_IT_SEES)

def convert_to_cv2_frame(name, image):
    data = image.getData()
    w = image.getWidth()
    h = image.getHeight()
    rows, cols = Config.get("dimensions")

    if name == "rgb":
        yuv = np.array(data).reshape((h * 3 // 2, w)).astype(np.uint8)
        frame = np.array(cv2.resize(cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12), (cols, rows)))
        return frame

    global last_rectif_right
    baseline = 75  # mm (example value)
    focal = right_intrinsic[0][0]
    max_disp = 96
    disp_type = np.uint8
    disp_levels = 1
    if extended:
        max_disp *= 2
    if subpixel:
        max_disp *= 32
        disp_type = np.uint16
        disp_levels = 32

    if name in ["depth", "bad_depth"]:
        disp = np.array(data).astype(np.uint8).view(disp_type).reshape((h, w))
        with np.errstate(divide='ignore'):
            frame = (disp_levels * baseline * focal / disp).astype(np.uint16)
        # frame = frame[:,40:] # for whatever reason, there is extra fov on the left??
        frame = np.asarray(Image.fromarray(frame).resize((cols, rows)))
        return frame
    
    # if name == "bad_depth":
    #     disp = np.array(data).astype(np.uint8).view(disp_type).reshape((h, w))
    #     frame = np.asarray(Image.fromarray(disp).resize((cols, rows)))
    #     return frame
    
def timeDeltaToMilliS(delta) -> float:
    return delta.total_seconds() * 1000

def oakd(rgbd_queue, bad_depth_queue, rgb_shared_mem, depth_shared_mem, bad_depth_shared_mem, still_running_queue, on_or_off):
    try:
        import depthai as dai
        
        dims = Config.get("dimensions")
        dummy_rgb = np.empty((dims[0], dims[1], 3), dtype=np.uint8)
        dummy_depth = np.empty((dims[0], dims[1]), dtype=np.uint16)

        rgb_buf = np.ndarray(dummy_rgb.shape, dtype=dummy_rgb.dtype, buffer=rgb_shared_mem.buf)
        depth_buf = np.ndarray(dummy_depth.shape, dtype=dummy_depth.dtype, buffer=depth_shared_mem.buf)
        bad_depth_buf = np.ndarray(dummy_depth.shape, dtype=dummy_depth.dtype, buffer=bad_depth_shared_mem.buf)

        while True:

            # Cant be in a separate func bc of weird depthai and multiprocessing interaction
            median    = dai.StereoDepthProperties.MedianFilter.KERNEL_7x7
            pipeline = dai.Pipeline()

            color_cam = pipeline.create(dai.node.ColorCamera)
            color_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            color_cam.setInterleaved(False)
            color_cam.setBoardSocket(dai.CameraBoardSocket.RGB)

            xout_rgb_video = pipeline.create(dai.node.XLinkOut)
            xout_rgb_video.setStreamName("rgb")

            color_cam.video.link(xout_rgb_video.input)
            
            mono_left = pipeline.create(dai.node.MonoCamera)
            mono_right = pipeline.create(dai.node.MonoCamera)
            mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
            mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
            mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
            mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

            stereo = pipeline.create(dai.node.StereoDepth)
            stereo.setConfidenceThreshold(200)
            stereo.setRectifyEdgeFillColor(0)
            stereo.initialConfig.setMedianFilter(median)
            stereo.setLeftRightCheck(lrcheck)
            stereo.setExtendedDisparity(extended)
            stereo.setSubpixel(subpixel)

            mono_left.out.link(stereo.left)
            mono_right.out.link(stereo.right)
            
            xout_left         = pipeline.create(dai.node.XLinkOut)
            xout_right        = pipeline.create(dai.node.XLinkOut)
            xout_disparity    = pipeline.create(dai.node.XLinkOut)
            xout_rectif_left  = pipeline.create(dai.node.XLinkOut)
            xout_rectif_right = pipeline.create(dai.node.XLinkOut)
            xout_left.setStreamName("left")
            xout_right.setStreamName("right")
            xout_disparity.setStreamName("depth")
            xout_rectif_left.setStreamName("rectified_left")
            xout_rectif_right.setStreamName("rectified_right")
            
            bad_stereo = pipeline.create(dai.node.StereoDepth)
            mono_left.out.link(bad_stereo.left)
            mono_right.out.link(bad_stereo.right)
            xout_bad_disparity    = pipeline.create(dai.node.XLinkOut)
            xout_bad_disparity.setStreamName("bad_depth")
            bad_stereo.disparity.link(xout_bad_disparity.input)

            stereo.syncedLeft.link(xout_left.input)
            stereo.syncedRight.link(xout_right.input)
            stereo.disparity.link(xout_disparity.input)
            stereo.rectifiedLeft.link(xout_rectif_left.input)
            stereo.rectifiedRight.link(xout_rectif_right.input)

            streams = ["rgb", "depth", "bad_depth", "left", "right", "rectified_left", "rectified_right"]

            with dai.Device(pipeline) as device:
                output_queues = {s: device.getOutputQueue(name=s, maxSize=4, blocking=False) for s in streams}

                rgb = None
                depth = None
                while True:
                    # try:
                        rgbframe = output_queues["rgb"].tryGet()
                        depthframe = output_queues["depth"].tryGet()
                        bad_depthframe = output_queues["bad_depth"].tryGet()
                        if on_or_off:
                            if rgbframe is not None:
                                rgb = convert_to_cv2_frame('rgb', rgbframe)
                            if depthframe is not None:
                                depth = convert_to_cv2_frame('depth', depthframe)
                            if rgb is not None and depth is not None:
                                rgb_buf[:] = rgb[:]
                                depth_buf[:] = depth[:]
                                rgbd_queue.put(0)
                                # plt.imshow(rgb)
                                # plt.savefig("figs/rgb.jpg", dpi=100)
                                # plt.close()
                                # plt.imshow(depth)
                                # plt.savefig("figs/depth.jpg", dpi=100)
                                # plt.close()
                                still_running_queue.put(0)
                                rgb = None
                                depth = None
                            if bad_depthframe is not None:
                                depth = convert_to_cv2_frame('bad_depth', bad_depthframe)
                                bad_depth_buf[:] = depth[:]
                                # plt.imshow(depth)
                                # plt.savefig("figs/bad_depth.jpg", dpi=100)
                                # plt.close()
                                bad_depth_queue.put(0)
    except RuntimeError:
        print("runtime error")
        still_running_queue.put(0)

def run_sca(sca, rgbd_queue, rgb_shared_mem, depth_shared_mem, steering_queue, steering_on):
    dims = Config.get("dimensions")
    rgb = np.empty((dims[0], dims[1], 3), dtype=np.uint8)
    depth = np.empty((dims[0], dims[1]), dtype=np.uint16)

    rgb_buf = np.ndarray(rgb.shape, dtype=rgb.dtype, buffer=rgb_shared_mem.buf)
    depth_buf = np.ndarray(depth.shape, dtype=depth.dtype, buffer=depth_shared_mem.buf)

    last_val = None
    while True:
        # try:
            while True:
                if not rgbd_queue.empty():
                    while not rgbd_queue.empty():
                        rgbd_queue.get()
                    rgb[:] = rgb_buf[:]
                    depth[:] = depth_buf[:]
                    steer = sca.add_np_array(np.copy(rgb), np.copy(depth), 0)
                    print("steering: " + str(steer))
                    if steer == last_val:
                        continue
                    last_val = steer
                    if steer is not None:
                        push_steer = "s" + str(steer)
                        # while not steering_queue.empty():
                        #     steering_queue.get()
                        steering_queue.put(push_steer)
                        steering_on = 1
                    # if SHOW_WHAT_IT_SEES:
                        # fig = sca.plot()
                        # plt.savefig("figs/sca.jpg", dpi=100)
                        # plt.close()
        # except:
        #     print("it broke:(")

def run_emergency_brake(bad_depth_queue, bad_depth_shared_mem, braking_queue, steering_on):
    dims = Config.get("dimensions")
    depth = np.empty((dims[0], dims[1]), dtype=np.uint16)
    bad_depth_buf = np.ndarray(depth.shape, dtype=depth.dtype, buffer=bad_depth_shared_mem.buf)

    last_val = None
    while True:
        # try:
            # while True:
                if not bad_depth_queue.empty(): 
                    while not bad_depth_queue.empty():
                        bad_depth_queue.get()
                    depth[:] = bad_depth_buf[:]
                    # plt.imshow(depth)
                    # plt.savefig("figs/bad_depth_for_brake.jpg", dpi=100)
                    # plt.close()
                    brake_val = ebrake.get_brake(depth)
                    print("brake: " + str(brake_val))
                    if brake_val == last_val or not steering_on:
                        continue
                    last_val = brake_val
                    push_brake = "b" + str(brake_val)
                    # while not braking_queue.empty():
                    #     braking_queue.get()
                    braking_queue.put(push_brake)
        # except:
        #     print("EB broke")

def send_socket(server_connection, steering_queue, braking_queue, on_or_off):

    def send(server_connection, steering_queue, braking_queue): # connect to outside server
        while True:
            # try: 
                        if not steering_queue.empty(): 
                            msg = steering_queue.get()
                            print(msg)
                            server_connection.sendall(msg.encode())
                        if not braking_queue.empty(): 
                            msg = braking_queue.get()
                            print(msg)
                            server_connection.sendall(msg.encode())
            # except:
            #     pass

    def receive(on_or_off): # receive outside client
        while True:
            # try: 
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.bind((HOST, SERVING_PORT))
                    while True:
                        s.listen()
                        conn, addr = s.accept()
                        with conn:
                            while True:
                                data = conn.recv(1024).decode()
                                if not data:
                                    continue
                                print("from client: " + data)
                                on_or_off = data
            # except:
            #     pass

    while True:
        # try:
            t1 = threading.Thread(target=send, args=(server_connection, steering_queue, braking_queue,))
            t2 = threading.Thread(target=receive, args=(on_or_off,))
            t1.start()
            t2.start()
            t1.join()
            t2.join()
        # except (ConnectionRefusedError, BrokenPipeError): # get rid
        #     pass

def rebooter(still_running_queue, on_or_off):
    time = None
    while True:
        if on_or_off:
            if not still_running_queue.empty(): 
                msg = still_running_queue.get()
                time = timer()
            if time is not None:
                if timer() - time > 1:
                    print("REBOOTING")
                    break

def run(sca, server_connection):
    try:
        rgbd_queue = multiprocessing.Queue()
        bad_depth_queue = multiprocessing.Queue()
        dims = Config.get("dimensions")
        dummy_rgb = np.empty((dims[0], dims[1], 3), dtype=np.uint8)
        dummy_depth = np.empty((dims[0], dims[1]), dtype=np.uint16)
        rgb_shared_mem = shared_memory.SharedMemory(create=True, size=dummy_rgb.nbytes)
        depth_shared_mem = shared_memory.SharedMemory(create=True, size=dummy_depth.nbytes)
        bad_depth_shared_mem = shared_memory.SharedMemory(create=True, size=dummy_depth.nbytes)
        steering_queue = multiprocessing.Queue()
        braking_queue = multiprocessing.Queue()
        still_running_queue = multiprocessing.Queue()
        on_or_off = multiprocessing.Value('i', 0)
        steering_on = multiprocessing.Value('i', 0)

        p1 = multiprocessing.Process(target=oakd, args=(rgbd_queue, bad_depth_queue, rgb_shared_mem, depth_shared_mem, bad_depth_shared_mem, still_running_queue, on_or_off,))
        p2 = multiprocessing.Process(target=run_sca, args=(sca, rgbd_queue, rgb_shared_mem, depth_shared_mem, steering_queue, steering_on,))
        p3 = multiprocessing.Process(target=run_emergency_brake, args=(bad_depth_queue, bad_depth_shared_mem, braking_queue, steering_on,))
        p4 = multiprocessing.Process(target=send_socket, args=(server_connection, steering_queue, braking_queue, on_or_off,))
        p5 = multiprocessing.Process(target=rebooter, args=(still_running_queue, on_or_off))

        p4.start()

        steering_queue.put("b1")

        p1.start()
        p2.start()
        p3.start()
        p5.start()

        p5.join()
        print("DSFHJKDSFHDSJKFHDSJKFHDSJKFHDSJKFHDSJKFHDSJKFHDSJKFHDHSJKFHDKJFHJKSKDHFJDSKJF")
        steering_queue.put("b1")
        p1.terminate()
        p1.join()
        p2.terminate()
        p2.join()
        p3.terminate()
        p3.join()
        p4.terminate()
        p4.join()
    except:
        print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
        p1.terminate()
        p1.join()
        p2.terminate()
        p2.join()
        p3.terminate()
        p3.join()
        p4.terminate()
        p4.join()
        p5.terminate()
        p5.join()
        server_connection.sendall("b1".encode())
