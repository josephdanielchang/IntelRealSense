import argparse
import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime
import time
import pandas
import csv

def main():
    if not os.path.exists(argument['depth_directory']):
        os.mkdir(argument['depth_directory'])
    if not os.path.exists(argument['rgb_directory']):
        os.mkdir(argument['rgb_directory'])
    if not os.path.exists(argument['csvfile']):
        with open(argument['csvfile'], 'wb') as csvfile:
            filewriter = csv.writer(csvfile, delimiter=',',
                                    quotechar='|', quoting=csv.QUOTE_MINIMAL)
    try:
        config = rs.config()
        rs.config.enable_device_from_file(config, argument['baginput'], False)
        pipeline = rs.pipeline()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)      ### CHANGE resolution and fps
        config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)     ### CHANGE resolution and fps
        config.enable_all_streams()
        profile = pipeline.start(config)
        playback = profile.get_device().as_playback()
        playback.set_real_time(False)
        i = 0
        while True:
            print("Saving frame:", i)
            frames = pipeline.wait_for_frames()

            # Align depth to color
            align = rs.align(rs.stream.color)
            frames = align.process(frames)
            
            for f in frames:
                print(f.profile)
            depth_frame = frames.get_depth_frame()              ## comment out if no depth
            rgb_frame   = frames.get_color_frame()
            
            depth_image = depth_frame.get_data()                ## comment out if no depth
            rgb_image   = rgb_frame.get_data()

            depth_image = np.asanyarray(depth_image)            ## comment out if no depth
            rgb_image   = np.asanyarray(rgb_image)
            
            bgr_image = rgb_image[..., ::-1]
            time_frame = frames.get_timestamp()
            time_utc = time_frame
            new_row = [i, time_utc]
            # Save timestamp as csv file and get png picture files
            with open(argument['csvfile'], 'a') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(new_row)
            csvfile.close()

            cv2.imwrite(argument['depth_directory'] + "/" + str(i).zfill(6) + "depth.png", depth_image)  ## comment out if no depth
            cv2.imwrite(argument['rgb_directory'] + "/" + str(i).zfill(6) + "rgb.png", bgr_image)
            i += 1
    finally:
        pass


if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # parser.add_argument("-d", "--directory", type=str, help="Path to save the images")
    # parser.add_argument("-i", "--input", type=str, help="Bag file to read")
    # args = parser.parse_args()
    argument = {}
    # CHANGE output depth directory
    argument['depth_directory'] = 'C:\\Users\\josep\\OneDrive\\Documents\\Research Fall 2019\\Processed Data\\intel_depth'
    # CHANGE output rgb directory
    argument['rgb_directory'] = 'C:\\Users\\josep\\OneDrive\\Documents\\Research Fall 2019\\Processed Data\\rgb'
    # CHANGE output timestamp directory
    argument['csvfile'] = 'C:\\Users\\josep\\OneDrive\\Documents\\Research Fall 2019\\Processed Data\\timestamps.csv'
    # CHANGE rosbag directory
    argument['baginput'] = '20191114_130938.bag'
    print(argument)
    main()
