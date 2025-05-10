#!/usr/bin/env python3
"""
This script creates a combined DepthAI pipeline that streams both RGB and stereo (depth) data.
It outputs:
  - RGB: "rgb_preview" and "rgb_video" streams from the ColorCamera.
  - Depth: "left", "right", "disparity", "rectified_left", and "rectified_right" streams from the stereo system.

Press 'q' to quit. Optionally, you can save frames as images.
"""

import cv2
import depthai as dai
import numpy as np
import argparse
import os
from time import sleep
import datetime


# ----------------- Global variables for Depth configuration -----------------
last_rectif_right = None
right_intrinsic = [[860.0, 0.0, 640.0],
                   [0.0, 860.0, 360.0],
                   [0.0, 0.0, 1.0]]
lrcheck   = True      # Better handling for occlusions
extended  = False     # Closer-in minimum depth, disparity range is doubled
subpixel  = False     # Better accuracy for longer distance, fractional disparity
median    = dai.StereoDepthProperties.MedianFilter.KERNEL_7x7
pcl_converter = None  # (Set this if you have a point cloud visualizer)
# -----------------------------------------------------------------------------

def create_combined_pipeline():
    """
    Creates a single pipeline that includes both:
      - An RGB ColorCamera (with preview and full video)
      - Two MonoCamera nodes feeding a StereoDepth node (outputting disparity, rectified images, etc.)
    
    Returns:
      pipeline: The DepthAI pipeline object.
      streams:  A list of stream names that will be output.
    """
    pipeline = dai.Pipeline()

    # ---------------------- Color (RGB) Section ----------------------
    color_cam = pipeline.create(dai.node.ColorCamera)
    color_cam.setPreviewSize(540, 540)
    color_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    color_cam.setInterleaved(False)
    color_cam.setBoardSocket(dai.CameraBoardSocket.RGB)

    # Create XLinkOut nodes for the RGB outputs.
    xout_rgb_preview = pipeline.create(dai.node.XLinkOut)
    xout_rgb_video   = pipeline.create(dai.node.XLinkOut)
    xout_rgb_preview.setStreamName("rgb_preview")
    xout_rgb_video.setStreamName("rgb_video")

    # Link ColorCamera outputs.
    color_cam.preview.link(xout_rgb_preview.input)
    color_cam.video.link(xout_rgb_video.input)
    # ------------------------------------------------------------------

    # ---------------------- Stereo / Depth Section --------------------
    # Create MonoCamera nodes for left and right.
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

    # Create the StereoDepth node.
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setConfidenceThreshold(200)
    stereo.setRectifyEdgeFillColor(0)  # Black edges.
    stereo.initialConfig.setMedianFilter(median)
    stereo.setLeftRightCheck(lrcheck)
    stereo.setExtendedDisparity(extended)
    stereo.setSubpixel(subpixel)

    # Link the mono camera outputs to the StereoDepth node.
    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)

    # Create XLinkOut nodes for the stereo outputs.
    xout_left         = pipeline.create(dai.node.XLinkOut)
    xout_right        = pipeline.create(dai.node.XLinkOut)
    xout_disparity    = pipeline.create(dai.node.XLinkOut)
    xout_rectif_left  = pipeline.create(dai.node.XLinkOut)
    xout_rectif_right = pipeline.create(dai.node.XLinkOut)
    xout_left.setStreamName("left")
    xout_right.setStreamName("right")
    xout_disparity.setStreamName("disparity")
    xout_rectif_left.setStreamName("rectified_left")
    xout_rectif_right.setStreamName("rectified_right")

    # Link StereoDepth outputs.
    stereo.syncedLeft.link(xout_left.input)
    stereo.syncedRight.link(xout_right.input)
    stereo.disparity.link(xout_disparity.input)
    stereo.rectifiedLeft.link(xout_rectif_left.input)
    stereo.rectifiedRight.link(xout_rectif_right.input)
    # ------------------------------------------------------------------

    # Return all stream names for later retrieval.
    streams = ["rgb_preview", "rgb_video",
               "left", "right", "disparity", "rectified_left", "rectified_right"]
    return pipeline, streams

def convert_to_cv2_frame(name, image):
    """
    Converts a DepthAI frame to a CV2 image.
    
    For RGB streams:
      - "rgb_video": The frame is in YUV NV12 and is converted to BGR.
      - "rgb_preview": The frame is in interleaved RGB format.
    
    For stereo streams:
      - "disparity": The frame is color-mapped.
      - "rectified_left", "rectified_right", "left", "right": Grayscale images.
    
    Returns:
      The converted image (numpy array) or None.
    """
    data = image.getData()
    w = image.getWidth()
    h = image.getHeight()

    if name == 'rgb_video':
        # Full-resolution video comes as YUV NV12.
        yuv = np.array(data).reshape((h * 3 // 2, w)).astype(np.uint8)
        frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
        return frame
    elif name == 'rgb_preview':
        # Preview comes as interleaved RGB.
        frame = np.array(data).reshape((3, h, w)).transpose(1, 2, 0).astype(np.uint8)
        return frame

    # --- For Stereo / Depth streams ---
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

    if name == 'depth':
        # (Not directly used here; the depth node outputs disparity.)
        frame = np.array(data).astype(np.uint8).view(np.uint16).reshape((h, w))
        return frame
    elif name == 'disparity':
        disp = np.array(data).astype(np.uint8).view(disp_type).reshape((h, w))
        # Optionally compute depth from disparity:
        with np.errstate(divide='ignore'):
            depth = (disp_levels * baseline * focal / disp).astype(np.uint16)
        frame = (disp * 255. / max_disp).astype(np.uint8)
        frame = cv2.applyColorMap(frame, cv2.COLORMAP_HOT)
        return frame
    elif name in ['rectified_left', 'rectified_right', 'left', 'right']:
        frame = np.array(data).reshape((h, w)).astype(np.uint8)
        if name == 'rectified_right':
            last_rectif_right = frame
        return frame
    else:
        return None


def main():
    parser = argparse.ArgumentParser(description="Combined RGB & Depth pipeline for DepthAI.")
    args = parser.parse_args()

    pipeline, streams = create_combined_pipeline()
    
    # Set the specific output directory
    base_output_folder = "" #file path of output data
    os.makedirs(base_output_folder, exist_ok=True)

    frame_counters = {}
    with dai.Device(pipeline) as device:
        output_queues = {s: device.getOutputQueue(name=s, maxSize=4, blocking=False) for s in streams}
        print("Running combined pipeline. Press 'q' to exit.")

        while True:
            for s in streams:
                in_frame = output_queues[s].tryGet()
                if in_frame is not None:
                    frame = convert_to_cv2_frame(s, in_frame)
                    if frame is not None:
                        cv2.imshow(s, frame)
                        
                        # Initialize counter for this stream if not exists
                        if s not in frame_counters:
                            frame_counters[s] = 0
                        frame_counters[s] += 1

                        # Save frame
                        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                        filename = f"{timestamp}_{s}_{frame_counters[s]:04d}.png"
                        stream_folder = os.path.join(base_output_folder, s)
                        os.makedirs(stream_folder, exist_ok=True)
                        
                        filepath = os.path.join(stream_folder, filename)
                        cv2.imwrite(filepath, frame)

            if cv2.waitKey(1) == ord('q'):
                break

        cv2.destroyAllWindows()



if __name__ == '__main__':
    main()
