// Plug left camera to left USB and right camera to right USB
// Build and run debugger to start recording
// Press stop debugging to stop recording
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
#include <Windows.h>

/*
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
*/

#include <cstdio>   //timer
#include <ctime>    //timer
std::clock_t start; //timer
double duration;    //timer

#define F_OK 0

using namespace cv;

double fps = 30;			// set frames per second to stream
double spf = 1 / fps;
bool pointcloud = true;	// set false to disable pointcloud
char buffer[50];
Mat image1_bgr;

void writeImages(rs2::frameset const& f, std::string& dir, int count, std::string cam) {
	std::stringstream path1, path2;
	// Set up path for images
	sprintf(buffer, "%05d.bmp", count);
	path1 << dir << "\\" << cam << "_rgb" << "\\" << "cam_" << cam << "_rgb_" << buffer;
	path2 << dir << "\\" << cam << "_intel_depth" << "\\" << "cam_" << cam << "_depth_" << buffer;
	// Create OpenCV image file, 8-bit, unsigned, 3 channels
	Mat image1(Size(640, 480), CV_8UC3, (void*)f.get_color_frame().get_data(), Mat::AUTO_STEP);
	// Transform color format
	cvtColor(image1, image1_bgr, COLOR_RGB2BGR);
	imwrite(path1.str(), image1_bgr);
	// Create OpenCV image file, 16-bit, unsigned, 1 channel
	Mat image2(Size(640, 480), CV_16UC1, (void*)f.get_depth_frame().get_data(), Mat::AUTO_STEP);
	imwrite(path2.str(), image2);
}
/*
using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
pcl_ptr points_to_pcl(const rs2::points& points)
{
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto ptr = points.get_vertices();
	for (auto& p : cloud->points)
	{
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}
	return cloud;
}
float3 colors[]{ { 0.8f, 0.1f, 0.3f },
				  { 0.1f, 0.9f, 0.5f },
};
// Declare pointcloud object for calculating pointclouds and texture mappings
rs2::pointcloud pc;
// We want the points object to be persistent so we can display the last cloud when a frame drops
rs2::points points;
void writePointcloud(rs2::frameset const& f, std::string& dir, int count, std::string cam) {
	std::stringstream path1, path2;
	// Set up path for images
	sprintf(buffer, "%05d.pcd", count);
	path1 << dir << "\\" << cam << "_pointcloud" << "\\" << "cam_" << cam << "_pointcloud_" << buffer;
	auto depth = f.get_depth_frame();
	auto points = pc.calculate(depth);
	// Transform it PCL point cloud like in the exemple rs - pcl
	ptr_cloud cloud = points_to_pcl(points);
	pcl::io::savePCDFile(path1, *cloud);
}
*/

int main(int argc, char* argv[]) try
{
	// Create a simple OpenGL window for rendering:
	window app(1280, 960, "Multi-Camera");

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
		cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480);
		cfg.enable_stream(RS2_STREAM_COLOR, 640, 480);
		// Start streaming
		pipe.start(cfg);
		pipelines.emplace_back(pipe);
	}
	// We'll keep track for the last frame of each stream available to make the presentation persistent
	std::map<int, rs2::frame> render_frames;
	// Define object to be used to align to depth to color stream
	rs2::align align_to_color(RS2_STREAM_COLOR);
	// Capture frames to give autoexposure, etc. a chance to settle
	for (auto i = 0; i < 100; ++i) {
		for (auto&& pipe : pipelines) pipe.wait_for_frames();
	}

	rs2::frameset fs;
	int count = 0;

	// Main app loop
	while (app) {
		start = std::clock();										  //timer
		// Collect the new frames from all the connected devices
		std::vector<rs2::frame> new_frames;
		for (auto&& pipe : pipelines) {
			rs2::frameset fs;
			// Use non-blocking frames polling method to minimize UI impact
			if (pipe.poll_for_frames(&fs)) {
				// Align newly-arrived frames to color viewport
				fs = align_to_color.process(fs);
				if (count > 0) {
					// Save processed frames and pointcloud to directories
					if (isLeft) {
						writeImages(fs, dirs[0], count, left);
						/*
						if (pointcloud) {
							writePointcloud(fs, dirs[0], count, left);
						}
						*/
					}
					else {
						writeImages(fs, dirs[1], count, right);
						/*
						if (pointcloud) {
							writePointcloud(fs, dirs[1], count, right);
						}
						*/
					}
				}
				isLeft = !isLeft;  // toggle is false then true
				// Split rs2::frameset containers into separate frames and store with standard C++ container for later use
				for (const rs2::frame& f : fs)
					new_frames.emplace_back(f);
			}
		}
		count++;
		// Convert the newly-arrived frames to render-friendly format
		for (const auto& frame : new_frames) {
			render_frames[frame.get_profile().unique_id()] = colorizer.process(frame);
		}
		// Present all the collected frames with openGl mosaic
		app.show(render_frames);
		duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;   //timer
		std::cout << "printf: " << duration << '\n';				  //timer
		Sleep((spf - duration)*1000);
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