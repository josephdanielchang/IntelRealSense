#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include "example.hpp"              // Include short list of convenience functions for OpenGL rendering

#include <map>
#include <vector>

#include <fstream>              // File IO
#include <iostream>             // Terminal IO
#include <sstream>              // Stringstreams

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

int main(int argc, char* argv[]) try
{
	// Create a simple OpenGL window for rendering:
	window app(1280, 960, "Multi-Camera");

	rs2::context                ctx;            // Create librealsense context for managing devices
	rs2::colorizer              colorizer;      // Utility class to convert depth data RGB colorspace
	std::vector<rs2::pipeline>  pipelines;

	texture depth_image, color_image;     // Helpers for rendering images

	// Start a streaming pipe per each connected device
	for (auto&& dev : ctx.query_devices())
	{
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
	for (auto i = 0; i < 30; ++i) 
	{
		for (auto&& pipe : pipelines) pipe.wait_for_frames();
	}
	
	// Main app loop
	while (app)
	{
		// Collect the new frames from all the connected devices
		std::vector<rs2::frame> new_frames;
		for (auto&& pipe : pipelines)
		{
			rs2::frameset fs;
			// Use non-blocking frames polling method to minimize UI impact
			if (pipe.poll_for_frames(&fs))		
			{
				// Align newly-arrived frames to color viewport
				fs = align_to_color.process(fs);

/*
				// We can only save video frames as pngs, so we skip the rest
				if (auto vf = fs.as<rs2::video_frame>())
				{
					// Write images to disk
					std::stringstream png_file;
					png_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name() << ".png";
					stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
						vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
					std::cout << "Saved " << png_file.str() << std::endl;
				}
*/

				// Split rs2::frameset containers into separate frames and store with standard C++ container for later use
				for (const rs2::frame& f : fs)
					new_frames.emplace_back(f);
			}
		}

		// Convert the newly-arrived frames to render-friendly format
		for (const auto& frame : new_frames)
		{
			render_frames[frame.get_profile().unique_id()] = colorizer.process(frame);
		}

		// Present all the collected frames with openGl mosaic
		app.show(render_frames);
	}

	return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
