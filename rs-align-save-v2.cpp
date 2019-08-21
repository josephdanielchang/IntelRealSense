// Plug left camera to left USB and right camera to right USB
// Build and run debugger to start recording
// Press stop debugging to stop recording
// Ensure left right frames in correct folders in final build with RAW_DATA false and CAM_SWITCHED
// Delete timer in final build

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
#define FPS 6							// SET camera fps (6,15,30,60)
#define FPS_MAX 200  					// SET max fps to stream (max stable fps varies), ignored by slower streams
#define DISPLAY_NEW_FPS_PER_FRAME false	// SET false to disable showing cutoff fps per frame on console output
#define DISPLAY_REG_FPS_PER_FRAME false	// SET false to disable showing regular fps per frame on console output
#define DISPLAY_FPS_PER_SECOND false	// SET false to disable showing fps per second on constole output
#define RGB_DEPTH_DIFF false			// SET false to disable showing rgb-depth-diff on console output
#define RAW_DATA false					// SET false to save rgb-depth as .bmp .png
#define TIMESTAMP true					// SET false to disable saving timestamps in .txt, time in seconds
#define CAM_SWITCHED true				// INVERT if left camera saving right images vice-versa
#define POINTCLOUD false				// SET false to disable pointcloud

int setSleep() {	// minimum sleep required to prevent duplicate frames and timestamps
	if (FPS == 6 && RAW_DATA)
		return 60;
	else if (FPS == 6 && !RAW_DATA)
		return 30;
	else if (FPS == 15 && RAW_DATA)
		return 20;
	else if (FPS == 15 && !RAW_DATA)
		return 0;
	else if (FPS == 30 && RAW_DATA)
		return 5;
	else if (FPS == 30 && !RAW_DATA)
		return 10;
	else if (FPS == 60 && RAW_DATA)
		return 0;
	else if (FPS == 60 && !RAW_DATA)
		return 0;
	else
		return EXIT_FAILURE;
}

char buffer[50];

void save_raw_data(rs2::frameset const& f, std::string& dir, int count, std::string cam) {
	auto depth = f.get_depth_frame();
	auto rgb = f.get_color_frame();
	// Set up path for images
	sprintf(buffer, "%05d", count);
	std::string path1(dir + "\\" + cam + "_rgb\\cam_" + cam + "_rgb_" + buffer + ".bin");
	std::string path2(dir + "\\" + cam + "_intel_depth\\cam_" + cam + "_intel_depth_" + buffer + ".bin");
	std::ofstream outfile1(path1.data(), std::ofstream::binary);
	std::ofstream outfile2(path2.data(), std::ofstream::binary);
	outfile1.write(static_cast<const char*>(rgb.get_data()), byte(rgb.get_height()) * rgb.get_stride_in_bytes());
	outfile2.write(static_cast<const char*>(depth.get_data()), byte(depth.get_height()) * depth.get_stride_in_bytes());
	outfile1.close();
	outfile2.close();
}

using namespace cv;
Mat image1_bgr;

void writeImages(rs2::frameset const& f, std::string& dir, int count, std::string cam) {
	std::stringstream path1, path2;
	// Set up path for images
	sprintf(buffer, "%05d", count);
	path1 << dir << "\\" << cam << "_rgb\\cam_" << cam << "_rgb_" << buffer << ".png";
	path2 << dir << "\\" << cam << "_intel_depth\\cam_" << cam << "_depth_" << buffer << ".png";
	// Create OpenCV image file, 8-bit, unsigned, 3 channels
	Mat image1(Size(WIDTH, HEIGHT), CV_8UC3, (void*)f.get_color_frame().get_data(), Mat::AUTO_STEP);
	// Transform color format
	cvtColor(image1, image1_bgr, COLOR_RGB2BGR);
	imwrite(path1.str(), image1_bgr);
	// Create OpenCV image file, 16-bit, unsigned, 1 channel
	Mat image2(Size(WIDTH, HEIGHT), CV_16UC1, (void*)f.get_depth_frame().get_data(), Mat::AUTO_STEP);
	imwrite(path2.str(), image2);
}

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

// Declare pointcloud object, for calculating pointclouds and texture mappings
rs2::pointcloud pc;
// We want the points object to be persistent so we can display the last cloud when a frame drops
rs2::points points;

void writePointCloud(rs2::frameset const& f, std::string& dir, int count, std::string cam) {
	// Set up path for pointclouds
	sprintf(buffer, "%05d", count);
	std::string path(dir + "\\" + cam + "_pointcloud\\cam_" + cam + "_pointcloud_" + buffer + ".ply");
	pc.map_to(f.get_color_frame());
	points = pc.calculate(f.get_depth_frame());
	points.export_to_ply(path, f.get_color_frame());
}

int main(int argc, char* argv[]) try
{
	// Create a simple OpenGL window for rendering:
	window app(640, 480, "Multi-Camera");

	rs2::context					ctx;            // Create librealsense context for managing devices
	rs2::colorizer					colorizer;      // Utility class to convert depth data RGB colorspace
	std::vector<rs2::pipeline>		pipelines;
	std::string dirs[] = { "left_data", "right_data" };
	std::string subdirs[] = { "rgb", "intel_depth", "pointcloud" };
	std::string path;
	bool isLeft = true;
	std::string left = "left";
	std::string right = "right";

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

	texture depth_image, color_image;     // Helpers for rendering images
	int mode = 1;						  // Initialize as master

	// Start a streaming pipe per each connected device
	for (auto&& device : ctx.query_devices()) {
		// Declare RealSense pipeline, encapsulating the actual device and sensors
		rs2::pipeline pipe(ctx);
		rs2::config cfg;
		rs2::sensor depth = device.query_sensors()[0];
		rs2::sensor color = device.query_sensors()[1];
		depth.set_option(RS2_OPTION_INTER_CAM_SYNC_MODE, mode);
		mode = 2;
		// Don't allow auto exposure to lower fps to let more light in
		color.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0.f);
		cfg.enable_device(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
		cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, rs2_format::RS2_FORMAT_Z16, FPS);
		cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, rs2_format::RS2_FORMAT_RGB8, FPS);
		// Start streaming
		pipe.start(cfg);
		pipelines.emplace_back(pipe);
	}
	// We'll keep track for the last frame of each stream available to make the presentation persistent
	std::map<int, rs2::frame> render_frames;
	// Define object to be used to align to depth to color stream
	rs2::align align_to_color(RS2_STREAM_COLOR);
	// Capture frames to give autoexposure, etc. a chance to settle
	for (auto i = 0; i < 30; ++i)
		for (auto&& pipe : pipelines) pipe.wait_for_frames();

	int count = -1;
	bool toggle = true;
	int counter = 0;
	double delay;
	double spf = 1.0 / FPS_MAX;
	std::clock_t start;			//timer
	double duration1;			//timer before sleep
	double duration2;			//timer after sleep
	double duration3 = 0.0;		//timer counting to 1 second
	int frames_this_second = 0; //counts frames to 1 second
	double rgb_t;				//timestamp
	double depth_t;				//timestamp

	int pause = setSleep();			// minimum sleep required to prevent duplicate frames and timestamps
	rs2::frameset fsLeft, fsRight;

	// Main app loop
	while (app) {
		//start = std::clock();										  //timer
		// Collect the new frames from all the connected devices
		std::vector<rs2::frame> new_frames;
		std::cout << "count:" << count << "\n";
		for (auto&& pipe : pipelines) {
			rs2::frameset fs;
			Sleep(pause);						
			// Use non-blocking frames polling method to minimize UI impact
			if (pipe.poll_for_frames(&fs)) {
				counter++;
				if (counter % 2 == 0) {
					count++;
					frames_this_second++;
				}
				if (!toggle) {
					fsRight = fs;
					toggle = true;
				}
				else {
					fsLeft = fs;
					toggle = false;
				}
			}
		}
		if (CAM_SWITCHED) {
			rs2::frameset temp = fsLeft;
			fsLeft = fsRight;
			fsRight = temp;
		}
		if (count > 0 && toggle == true) { // so left an right pipes polled already
			//LEFT CAMERA
			// Align newly-arrived frames to color viewport
			fsLeft = align_to_color.process(fsLeft);
			if (TIMESTAMP || RGB_DEPTH_DIFF) {
				rgb_t = fsLeft.get_color_frame().get_timestamp();
				depth_t = fsLeft.get_depth_frame().get_timestamp();
			}
			if (RGB_DEPTH_DIFF) {
				std::cout << "diff (ms) :" << std::setprecision(15) << rgb_t - depth_t << "\n";
			}
			// Save processed frames and pointcloud to directories
			if (RAW_DATA)
				save_raw_data(fsLeft, dirs[0], count, left);
			else
				writeImages(fsLeft, dirs[0], count, left);
			if (TIMESTAMP)
				save_timestamp(dirs[0], count, left, rgb_t, depth_t);
			if (POINTCLOUD)
				writePointCloud(fsLeft, dirs[0], count, left);
			// Split rs2::frameset containers into separate frames and store with standard C++ container for later use
			for (const rs2::frame& f : fsLeft)
				new_frames.emplace_back(f);
			//RIGHT CAMERA
			// Align newly-arrived frames to color viewport
			fsRight = align_to_color.process(fsRight);
			if (TIMESTAMP || RGB_DEPTH_DIFF) {
				rgb_t = fsRight.get_color_frame().get_timestamp();
				depth_t = fsRight.get_depth_frame().get_timestamp();
			}
			if (RGB_DEPTH_DIFF) {
				std::cout << "diff (ms) :" << std::setprecision(15) << rgb_t - depth_t << "\n";
			}
			// Save processed frames and pointcloud to directories
			if (RAW_DATA)
				save_raw_data(fsRight, dirs[1], count, right);
			else
				writeImages(fsRight, dirs[1], count, right);
			if (TIMESTAMP)
				save_timestamp(dirs[1], count, right, rgb_t, depth_t);
			if (POINTCLOUD)
				writePointCloud(fsRight, dirs[1], count, right);
			// Split rs2::frameset containers into separate frames and store with standard C++ container for later use
			for (const rs2::frame& f : fsRight)
				new_frames.emplace_back(f);
		}
		// Convert the newly-arrived frames to render-friendly format
		for (const auto& frame : new_frames)
			render_frames[frame.get_profile().unique_id()] = colorizer.process(frame);
		// Present all the collected frames with openGl mosaic
		app.show(render_frames);
		/*
		duration1 = (std::clock() - start) / (double)CLOCKS_PER_SEC;       //timer
		if (DISPLAY_REG_FPS_PER_FRAME)
			std::cout << "reg fps: " << 1.0 / duration1 << '\n';		   //timer
		if (spf > duration1) {
			delay = spf - duration1;
			Sleep(delay * 1000.0);
		}
		duration2 = (std::clock() - start) / (double)CLOCKS_PER_SEC;	   //timer
		if (DISPLAY_NEW_FPS_PER_FRAME)
			std::cout << "new fps:" << 1.0 / duration2 << '\n';			   //timer
		if (DISPLAY_FPS_PER_SECOND) {
			duration3 += duration2;
			if (duration3 >= 1.0) {
				std::cout << "fps: " << frames_this_second << '\n';
				duration3 = 0.0;
				frames_this_second = 0;
			}
		}
		*/
	}
	return EXIT_SUCCESS;
}
catch (const rs2::error& e) {
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e) {
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
