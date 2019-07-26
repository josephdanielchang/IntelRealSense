// Plug left camera to left USB and right camera to right USB

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

#define F_OK 0

using namespace cv;

void writeImages(rs2::frameset f, std::string& dir, int count, std::string cam) {
	// Set up path for images
	std::stringstream path1, path2, p;
	char buffer[50];
	sprintf(buffer, "cam_%s_depth_%05d.png", cam, count);
	path1 << dir << "\\" << cam << "_intel_rgb" << "\\" << "cam_" << cam << "_rgb" << buffer;
	path2 << dir << "\\" << cam << "_intel_depth" << "\\" << "cam_" << cam << "_depth_" << buffer;
	
	// Create OpenCV image file
	Mat image1(Size(640, 480), CV_8UC3, (void*)f.get_color_frame().get_data(), Mat::AUTO_STEP);
	Mat image1_copy;

	// Transform color format
	cvtColor(image1, image1_copy, COLOR_RGB2BGR);

	imwrite(path1.str(), image1_copy);

	// Create OpenCV image file
	Mat image2(Size(640, 480), CV_16UC1, (void*)f.get_depth_frame().get_data(), Mat::AUTO_STEP);

	imwrite(path2.str(), image2);
	
	// Console output
	std::cout << "Saved " << path2.str() << ' ' << f.get_frame_number() << std::endl;
}

int main(int argc, char* argv[]) try
{
	// Create a simple OpenGL window for rendering:
	window app(1280, 960, "Multi-Camera");

	rs2::context					ctx;            // Create librealsense context for managing devices
	rs2::colorizer					colorizer;      // Utility class to convert depth data RGB colorspace
	std::vector<rs2::pipeline>		pipelines;
	std::string dirs[]				= { "left_data", "right_data" };
	std::string subdirs[]			= { "rgb", "intel_depth" };
	std::string path;
	bool isLeft						= true;
	int count = 1;

	// Create directories to store processed frames
	for (auto dir : dirs) {									// for 2 directories
		if ((_access(dir.c_str(), F_OK))) {					// check if directory exists
			_mkdir(dir.c_str());							// create directory
		}
		for (auto subdir : subdirs) {						// for 2 subdirectories
			if (isLeft) {
				path = dir + "\\left_" + subdir;			// subdirectory path
			}
			else {
				path = dir + "\\right_" + subdir;			// subdirectory path
			}
			if (_access(path.c_str(), F_OK)) {				// check if path exists
				_mkdir(path.c_str());						// create subdirectory
			}
		}
		isLeft = !isLeft;  // toggle is false then true
	}

	texture depth_image, color_image;     // Helpers for rendering images

	// Start a streaming pipe per each connected device
	for (auto&& dev : ctx.query_devices()) {
		// Declare RealSense pipeline, encapsulating the actual device and sensors
		rs2::pipeline pipe(ctx);
		rs2::config cfg;
		cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
		cfg.enable_stream(RS2_STREAM_DEPTH);
		cfg.enable_stream(RS2_STREAM_COLOR);
		// Start streaming
		pipe.start(cfg);
		pipelines.emplace_back(pipe);
	}

	// We'll keep track for the last frame of each stream available to make the presentation persistent
	std::map<int, rs2::frame> render_frames;

	// Define object to be used to align to depth to color stream
	rs2::align align_to_color(RS2_STREAM_COLOR);

	// Capture 30 frames to give autoexposure, etc. a chance to settle
	for (auto i = 0; i < 30; ++i) {
		for (auto&& pipe : pipelines) pipe.wait_for_frames();
	}

	// Main app loop
	while (app) {
		// Collect the new frames from all the connected devices
		std::vector<rs2::frame> new_frames;
		for (auto&& pipe : pipelines) {
			rs2::frameset fs;
			// Use non-blocking frames polling method to minimize UI impact
			if (pipe.poll_for_frames(&fs)) {
				// Align newly-arrived frames to color viewport
				fs = align_to_color.process(fs);

				// Save processed frames to directories
				if (isLeft) {
					writeImages(fs[0], dirs[0], count, "left");
					writeImages(fs[1], dirs[0], count, "left");
				}
				else {
					writeImages(fs[0], dirs[1], count, "right");
					writeImages(fs[1], dirs[1], count, "right");
				}
				isLeft = !isLeft;  // toggle is false then true

				// Split rs2::frameset containers into separate frames and store with standard C++ container for later use
				for (const rs2::frame& f : fs)
					new_frames.emplace_back(f);
			}
		}
		
		// Convert the newly-arrived frames to render-friendly format
		for (const auto& frame : new_frames) {
			render_frames[frame.get_profile().unique_id()] = colorizer.process(frame);
		}

		// Present all the collected frames with openGl mosaic
		app.show(render_frames);
		
		count++;
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