#!/usr/bin/env python3

import numpy as np
import pyrealsense2 as rs
import h5py
import time

def capture_rgbd():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    
    if not depth_frame or not color_frame:
        raise RuntimeError("Could not acquire depth or color frames.")

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Stop streaming
    pipeline.stop()

    return color_image, depth_image

def save_to_hdf5(rgb, depth, filename='output.h5'):
    with h5py.File(filename, 'w') as f:
        f.create_dataset('rgb', data=rgb)
        f.create_dataset('depth', data=depth)

def main():
    while True:
        rgb, depth = capture_rgbd()
        save_to_hdf5(rgb, depth)
        print("Saved RGB-D data to output.h5")
        time.sleep(1)  # Adjust delay as needed

if __name__ == "__main__":
    main()

