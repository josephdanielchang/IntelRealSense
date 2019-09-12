#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_internal.hpp>
#include <opencv2/opencv.hpp>
#include "../example.hpp"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>
#include <int-rs-splash.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstring>

#define WIDTH 640						// SET stream width
#define HEIGHT 480						// SET stream height
#define BPP 2							// SET 16 bit unsigned
#define FPS 30

struct synthetic_frame
{
	int x, y, bpp;
	std::vector<uint8_t> frame;
};

class custom_frame_source
{
public:
	custom_frame_source()
	{
		depth_frame.x = WIDTH;
		depth_frame.y = HEIGHT;
		depth_frame.bpp = BPP;

		last = std::chrono::high_resolution_clock::now();

		std::vector<uint8_t> pixels_depth(depth_frame.x * depth_frame.y * depth_frame.bpp, 0);
		depth_frame.frame = std::move(pixels_depth);

		auto realsense_logo = stbi_load_from_memory(splash, (int)splash_size, &color_frame.x, &color_frame.y, &color_frame.bpp, false);

		std::vector<uint8_t> pixels_color(color_frame.x * color_frame.y * color_frame.bpp, 0);

		memcpy(pixels_color.data(), realsense_logo, color_frame.x * color_frame.y * 4);

		for (auto i = 0; i < color_frame.y; i++)
			for (auto j = 0; j < color_frame.x * 4; j += 4)
			{
				if (pixels_color.data()[i * color_frame.x * 4 + j] == 0)
				{
					pixels_color.data()[i * color_frame.x * 4 + j] = 22;
					pixels_color.data()[i * color_frame.x * 4 + j + 1] = 115;
					pixels_color.data()[i * color_frame.x * 4 + j + 2] = 185;
				}
			}
		color_frame.frame = std::move(pixels_color);
	}

	synthetic_frame& get_synthetic_texture()
	{
		return color_frame;
	}

	synthetic_frame& get_synthetic_depth(glfw_state& app_state)
	{
		draw_text(50, 50, "This point-cloud is generated from a synthetic device:");

		auto now = std::chrono::high_resolution_clock::now();
		if (now - last > std::chrono::milliseconds(1))
		{
			app_state.yaw -= 1;
			wave_base += 0.1f;
			last = now;

			for (int i = 0; i < depth_frame.y; i++)
			{
				for (int j = 0; j < depth_frame.x; j++)
				{
					auto d = 2 + 0.1 * (1 + sin(wave_base + j / 50.f));
					((uint16_t*)depth_frame.frame.data())[i * depth_frame.x + j] = (int)(d * 0xff);
				}
			}
		}
		return depth_frame;
	}

	rs2_intrinsics create_texture_intrinsics()
	{
		rs2_intrinsics intrinsics = { color_frame.x, color_frame.y,
			(float)color_frame.x / 2, (float)color_frame.y / 2,
			(float)color_frame.x / 2, (float)color_frame.y / 2,
			RS2_DISTORTION_BROWN_CONRADY ,{ 0,0,0,0,0 } };

		return intrinsics;
	}

	rs2_intrinsics create_depth_intrinsics()
	{
		rs2_intrinsics intrinsics = { depth_frame.x, depth_frame.y,
			(float)depth_frame.x / 2, (float)depth_frame.y / 2,
			(float)depth_frame.x , (float)depth_frame.y ,
			RS2_DISTORTION_BROWN_CONRADY ,{ 0,0,0,0,0 } };

		return intrinsics;
	}

private:
	synthetic_frame depth_frame;
	synthetic_frame color_frame;

	std::chrono::high_resolution_clock::time_point last;
	float wave_base = 0.f;
};

int main()
{
	int frame_number = 0;
	custom_frame_source app_data;

	auto texture = app_data.get_synthetic_texture(); //REPLACE get rgb.bin file

	rs2_intrinsics color_intrinsics = app_data.create_texture_intrinsics();
	rs2_intrinsics depth_intrinsics = app_data.create_depth_intrinsics();

	//==================================================//
	//           Declare Software-Only Device           //
	//==================================================//
	
	// Object functions as converter from raw data into SDK objects and allows us to pass our synthetic images into SDK algorithms
	rs2::software_device dev; // Create software-only device

    auto depth_sensor = dev.add_sensor("Depth"); // Define depth sensor
    auto color_sensor = dev.add_sensor("Color"); // Define rgb sensor

	// Before can pass images into device, must provide details about the stream to simulate (stream type, dimensions, formatand stream intrinsics
	// To properly simulate depth data, must define legal intrinsics that will be used to project pixels into 3D space

	auto depth_stream = depth_sensor.add_video_stream({ RS2_STREAM_DEPTH, 0, 0,
								WIDTH, HEIGHT, FPS, BPP,
								RS2_FORMAT_Z16, depth_intrinsics });

	// Required for point cloud generation
	depth_sensor.add_read_only_option(RS2_OPTION_DEPTH_UNITS, 0.001f);


	auto color_stream = color_sensor.add_video_stream({ RS2_STREAM_COLOR, 0, 1, texture.x,
								texture.y, FPS, texture.bpp,
								RS2_FORMAT_RGBA8, color_intrinsics });

	dev.create_matcher(RS2_MATCHER_DLR_C);
	rs2::syncer sync;

	depth_sensor.open(depth_stream);
	color_sensor.open(color_stream);

	depth_sensor.start(sync);
	color_sensor.start(sync);

	depth_stream.register_extrinsics_to(color_stream, { { 1,0,0,0,1,0,0,0,1 },{ 0,0,0 } });

	/*
	int count = 0;
	char buffer[50];
	std::string path1, path2, path3, path4, frame;

	while (1) { // change to count = 0 to 1 later
		
		count++;

		//INPUT FILE
		sprintf(buffer, "%05d", count);
		std::string path1("C:\\Program Files(x86)\\Intel RealSense SDK 2.0\\samples\\ar-basic\\left_rgb\\cam_left_rgb_" + buffer + ".bin");
	
		std::ifstream infile1, infile2, infile3, infile4;
		infile1.open(path1, std::ifstream::binary);
		// get size of file
		infile1.seekg(0, infile1.end);
		long size = infile1.tellg();
		infile1.seekg(0);
		// allocate memory for file content
		char* buffer = new char[size];
		// read content of infile
		infile1.read(buffer, size);

		//std::cout << "working" << count;

		// OUTPUT FILE
		path1 = "C:\\Program Files(x86)\\Intel RealSense SDK 2.0\\samples\\ar-basic\\left_rgb_png\\cam_left_rgb_" + frame + ".png";

		std::ofstream outfile1, outfile2, outfile3, outfile4;
		outfile1.open(path1, std::ofstream::binary);

		// write to outfile
		outfile1.write(buffer, size);
		// release dynamically-allocated memory
		delete[] buffer;

		outfile1.close();
		infile1.close();
		
		outfile1.write(static_cast<rs2::frame>(infile1);
		outfile2.write(static_cast<rs2::frame>(infile2);
		outfile3.write(static_cast<rs2::frame>(infile3);
		outfile4.write(static_cast<rs2::frame>(infile4);
		
		outfile1.close();
		outfile2.close();
		outfile3.close();
		outfile4.close();
	}
	*/

}