// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>
#include "example.hpp"              // Include short list of convenience functions for OpenGL rendering
#include <fstream>					// File IO
#include <iostream>					// Terminal IO
#include <sstream>					// Stringstreams
#include <direct.h>					// System directories
#include <io.h>						// EOF (end of file) is global constant returning -1
#include <vector>
#include <map>
#include <Windows.h>				//sleep
#include <cstdio>					//timer
#include <ctime>					//timer

#define F_OK 0
#define WIDTH 640						// SET stream width
#define HEIGHT 480						// SET stream height
#define FPS 30							// SET camera fps (6,15,30,60)
#define TIMESTAMP true					// SET false to disable saving timestamps in .txt, time in seconds
#define CAM_SWITCHED true				// INVERT if left camera saving right images vice-versa

void save_timestamp(std::string& dir, int count, std::string cam, double rgb_t, double depth_t) {
	// Set up path for timestamps
	std::string path1(dir + "\\cam_" + cam + "_rgb_timestamp.txt");
	std::string path2(dir + "\\cam_" + cam + "_intel_depth_timestamp.txt");
	std::ofstream outfile1;
	std::ofstream outfile2;
	outfile1.open(path1, std::ios_base::app);
	outfile2.open(path2, std::ios_base::app);
	outfile1 << count << ", " << std::setprecision(25) << rgb_t << "\n";
	outfile2 << count << ", " << std::setprecision(25) << depth_t << "\n";
	outfile1.close();
	outfile2.close();
}

int main(int argc, char* argv[]) try
{
	// Create a simple OpenGL window for rendering:
	window app(640, 480, "CPP Multi-Camera Example");

	rs2::context                          ctx;        // Create librealsense context for managing devices
	std::map<std::string, rs2::colorizer> colorizers; // Declare map from device serial number to colorizer (utility class to convert depth data RGB colorspace)
	std::vector<rs2::pipeline>            pipelines;


	// We'll keep track of the last frame of each stream available to make the presentation persistent
	std::map<int, rs2::frame> render_frames;

	std::string dirs[] = { "left_data", "right_data" };
	std::string subdirs[] = { "rgb", "intel_depth", "pointcloud" };
	std::string path;
	std::string left = "left";
	std::string right = "right";

	int frameNum = 0;
	double rgb_t;				//timestamp
	double depth_t;				//timestamp
	bool isLeft = true;


	// Create directories to store processed frames
	for (auto dir : dirs) {									// for 2 directories
		if ((_access(dir.c_str(), F_OK)))					// check if directory exists
			_mkdir(dir.c_str());							// create directory
		for (auto subdir : subdirs) {						// for 2 subdirectories
			if (isLeft)
				path = dir + "\\left_" + subdir;			// subdirectory path
			else
				path = dir + "\\right_" + subdir;			// subdirectory path
			if (_access(path.c_str(), F_OK))				// check if path exists
				_mkdir(path.c_str());						// create subdirectory
		}
		isLeft = !isLeft;  // toggle is false then true
	}

	if (CAM_SWITCHED)
		isLeft = !isLeft;

	int mode = 1;
	// Start a streaming pipe per each connected device
	for (auto&& dev : ctx.query_devices())
	{
		// Declare RealSense pipeline, encapsulating the actual device and sensors
		rs2::pipeline pipe(ctx);
		rs2::config cfg;

		rs2::sensor depth = dev.query_sensors()[0];
		rs2::sensor color = dev.query_sensors()[1];
		depth.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, mode);
		mode = 2;
		
		// Allow sensor to dynamically adjust fps depending on lighting conditions
		//color.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0.f);

		// Enable image auto exposure (keeps fps from lowering for some reason)
		depth.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
		color.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);

		cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
		cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, rs2_format::RS2_FORMAT_Z16, FPS);
		cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, rs2_format::RS2_FORMAT_RGB8, FPS);
		// Start streaming
		pipe.start(cfg);
		pipelines.emplace_back(pipe);
		// Map from each device's serial number to a different colorizer
		colorizers[dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)] = rs2::colorizer();
	}

	// Define object to be used to align to depth to color stream
	rs2::align align_to_color(RS2_STREAM_COLOR);

	// Main app loop
	while (app)
	{
		// Collect the new frames from all the connected devices
		std::vector<rs2::frame> new_frames;
		for (auto&& pipe : pipelines)
		{
			rs2::frameset fs;
			if (pipe.poll_for_frames(&fs))
			{
				// Align newly-arrived frames to color viewport
				fs = align_to_color.process(fs);

				// Save frame timestamps to directories
				if (TIMESTAMP) {
					rgb_t = fs.get_color_frame().get_timestamp();
					depth_t = fs.get_depth_frame().get_timestamp();
					if (isLeft)
						save_timestamp(dirs[0], frameNum, left, rgb_t, depth_t);
					else
						save_timestamp(dirs[1], frameNum, right, rgb_t, depth_t);
				}
				
				isLeft = !isLeft;

				for (const rs2::frame& f : fs)
					new_frames.emplace_back(f);
			}
		}

		frameNum++;

		// Convert the newly-arrived frames to render-friendly format
		for (const auto& frame : new_frames)
		{
			// Get the serial number of the current frame's device
			auto serial = rs2::sensor_from_frame(frame)->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
			// Apply the colorizer of the matching device and store the colorized frame
			render_frames[frame.get_profile().unique_id()] = colorizers[serial].process(frame);
		}

		// Present all the collected frames with openGl mosaic
		app.show(render_frames);
	}

	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}