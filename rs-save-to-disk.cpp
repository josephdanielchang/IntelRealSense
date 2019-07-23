// run this script before recording 
// https://github.com/IntelRealSense/librealsense/blob/master/scripts/realsense_metadata_win10.ps1

#include <librealsense2/rs.hpp> // RealSense Cross Platform API, all rs2 namespace
#include <opencv2/opencv.hpp>   // OpenCV
#include <fstream>              // File IO
#include <iostream>             // Terminal IO
#include <sstream>              // Stringstreams
#include <cmath>                // Trig

#include <direct.h>				// System directories
#include <vector>				// Vector array
#include <map>                  // Map values
//#include <Windows.h>
#include <io.h>					// EOF (end of file) is global constant returning -1
#define F_OK 0					//
//#include "example.hpp"

using namespace rs2;
using namespace std;
using namespace cv;
const double thres = 16.666;	//time allowed between each frame
int tm; 

// loadbagfile: load a bag file to a pipeline
void loadbagfile(shared_ptr<pipeline> pipe, std::string filename)
{
  rs2::config cfg;
  cfg.enable_device_from_file(filename);									// enable device from cam_#.bag
  pipe->start(cfg);															// start pipeline streaming according to the configuraion.

  auto device = pipe->get_active_profile().get_device();
  auto const i = pipe->get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
  
  rs2::playback playback = device.as<rs2::playback>();
  playback.set_real_time(false); 											// playback frame by frame not real time
}

// readframe: read the next frame of a bag-file-playback-pipeline
int readframe(shared_ptr<pipeline> pipe, size_t & framenumber, rs2::frameset & frameset)	// pipe, framenumbers, frameset
{

  frameset = pipe->wait_for_frames();										// wait until new set of frames available

  if (frameset[0].get_frame_number() < framenumber) {						// frame number of the frame, in milliseconds since the device was started
    return EOF; 
  } 
  
  if (tm >= 10) return EOF; // prevent repeat frames at the end

  framenumber = frameset[0].get_frame_number();

  return 0;
}

// readframeto: read the frame just after the specified timestamp.  
int readframeto(long long timestamp, shared_ptr<pipeline> pipe, size_t & framenumber, rs2::frameset & frameset)
{
  size_t prev = timestamp -
    frameset.get_frame_metadata((rs2_frame_metadata_value)RS2_FRAME_METADATA_TIME_OF_ARRIVAL);
  
  while (true)
  { 
    if (abs(prev) <= thres)
      return 0;

    int r = readframe(pipe, framenumber, frameset);
    size_t curr = timestamp -
      frameset.get_frame_metadata((rs2_frame_metadata_value)RS2_FRAME_METADATA_TIME_OF_ARRIVAL);
    
    if (r == EOF || curr > prev) return EOF; // stop when goes back to the start point
    
    prev = curr; 
  }

  return 0;
}


void writeImages(frameset & f, std::string & dir, int count) {
  
  // set up path for images
  std::stringstream pathd, pathc,p;
  char buffer[50]; 
  sprintf(buffer, "%05d.png", count); 
  char b2[50]; 
  sprintf(b2, "%d.csv", count); 
  pathd << dir << "\\" << "depth" << "\\" << dir << "_depth_" << "frame" << "_" << buffer; 
  pathc << dir << "\\" << "rgb" << "\\" << dir << "_rgb_" << "frame" << "_" << buffer; 
  p << dir << "\\" << "depth" << "\\" << dir << "_depth_" << "frame" << "_" << b2;

  // create opencv image file
  Mat image1(Size(640, 480), CV_8UC3, (void*)f.get_color_frame().get_data(), Mat::AUTO_STEP);
  Mat image1_copy; 
  
  // transform color format
  cvtColor(image1, image1_copy, COLOR_RGB2BGR);
  
  imwrite(pathc.str(), image1_copy);

  Mat image2(Size(640, 480), CV_16UC1, (void*)f.get_depth_frame().get_data(), Mat::AUTO_STEP);
  
  imwrite(pathd.str(), image2);
  std::cout << "Saved " << pathd.str() << ' ' << f.get_frame_number() << std::endl;
  
}

int main()
{
  const int num_file = 2;

  std::vector<shared_ptr<pipeline>> pipes;						// (vector) pipes
  std::vector<size_t> framenumbers;								// (vector) framenumbers
  std::vector<rs2::frameset> framesets;							// (vector) framesets
  int calibration = 0; 											// CHANGE: set to 1 for extracting calibration frames
  string files[] = { "cam_", "calib_" }; 
  std::string filename = files[calibration]; 					// (string) filename "cam_" or "calib_"
  std::string filetype = ".bag";								// (string) filetype ".bag"

  std::string bagfiles[num_file];								// (string) bagfiles size 2
  std::string dirs[num_file];									// (string) dirs 	 size 2
  std::string subdirs[] = { "ir", "rgb", "depth" };				// (string) subdirs  "ir", "rgb", "depth"

  for (int i = 0; i < num_file; i++) {							// for 2 cameras
    bagfiles[i] = filename + std::to_string(i + 1) + filetype;	// name the bag file ie. cam_1.bag, cam_2.bag
  }

  rs2::align align_to_color(RS2_STREAM_COLOR);					// align depth to color stream (rgb camera)
  
  // step 1: load bag files 
  pipes.clear();												// clear pipes 
  for (int i = 0; i < num_file; i++)							// for 2 cameras
  {
    //rs2::pipeline pipe;
    auto pipe = std::make_shared<rs2::pipeline>(); 				// object pipe owns/stores pointer to it, create pipeline for camera streaming
    loadbagfile(pipe, bagfiles[i]);								// LOADBAGFILE FUNCTION: playback bag files
    pipes.push_back(pipe);										// add pipe to end of pipes
    framenumbers.push_back(0ULL);								// add 0 (unsigned long long) to end of framenumbers
  }
  // step 1 end 

  
  // create directories to store processed frames
  for (int i = 0; i < num_file; i++) {																	// for 2 cameras
    string dig = pipes[i]->get_active_profile().get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);	// get camera serial number
    dig = dig.substr(dig.length() - 5); 																// delete last 5 string characters
    dirs[i] = filename + std::to_string(i + 1) + "_" + dig; 											// name directories ie. cam_1_serialnumber
  }

  for (auto dir : dirs) {										// for 2 dir		

    if ((_access(dir.c_str(), F_OK))) {							// check if dir already exists
      _mkdir(dir.c_str());										// make dir directory
    }

    for (auto subdir : subdirs) {								// for 3 subdir
      std::string loc = dir + "\\" + subdir;					// (string) loc	"dir/subdir"

      if ((_access(loc.c_str(), F_OK))) {						// check if loc already exists
        _mkdir(loc.c_str());									// make loc directory
      }

    }

  }

  // step 2: skip frames until the time when every camera is ready (t_start). ///
  for (int j = 0; j < 50; j++) {								// skip first 50 frames
    rs2::frameset frameset;										// extends frame class with frameset attributes
    readframe(pipes[0], framenumbers[0], frameset);				// read frame from cam 1 serial number, frame numbers, object
    readframe(pipes[1], framenumbers[1], frameset);				// read frame from cam 2 serial number, frame numbers, object
  }

  for (int i = 0; i < num_file; i++)							// for 2 cameras
  {
    rs2::frameset frameset;										// extends frame class with frameset attributes
    readframe(pipes[i], framenumbers[i], frameset);				// read next frame of pipeline (cam i serial number, frame numbers, object)
    framesets.push_back(frameset);								// add another frameset to end of framesets vector
  }



  long long t_start = framesets[0].get_frame_metadata((rs2_frame_metadata_value)RS2_FRAME_METADATA_TIME_OF_ARRIVAL); // cam 1 frame arrival time
  long long t_1 = framesets[1].get_frame_metadata((rs2_frame_metadata_value)RS2_FRAME_METADATA_TIME_OF_ARRIVAL);	 // cam 2 frame arrival time
  
  cerr << t_start << endl;											// error stream cam 1 frame arrival time
  cerr << t_1 << endl;												// error stream cam 2 frame arrival time
  //cerr << timestamp << " " << timestamp2 << endl;

  if (t_start < t_1) t_start = t_1;									// if cam 1 frame earlier than cam 2 frame, equalize to cam 2


  for (int i = 0; i < num_file; i++)								// for 2 cameras
  {
    readframeto(t_start, pipes[i], framenumbers[i], framesets[i]);	// read frame after specified timestamp (t_start)
  }

  // step 2 end /////////////////////////////////////////////////////////////////
  int count = 1;
  int r;
  
  do {
    for (int i = 0; i < num_file; i++) {							// for 2 cameras
      r = readframe(pipes[i], framenumbers[i], framesets[i]);		// read next frame of pipeline
      //cerr << framesets[i].get_frame_metadata((rs2_frame_metadata_value)RS2_FRAME_METADATA_BACKEND_TIMESTAMP) << endl; 
      if (r == EOF) goto end;										// r is equal to last frame
      
    }
    long long t0 = framesets[0].get_frame_metadata((rs2_frame_metadata_value)RS2_FRAME_METADATA_TIME_OF_ARRIVAL);	// cam 1 frame arrival time
    long long t1 = framesets[1].get_frame_metadata((rs2_frame_metadata_value)RS2_FRAME_METADATA_TIME_OF_ARRIVAL);	// cam 2 frame arrival time
    long long diff = t0 - t1;																						// calculate lag

    // for time lag larger than threshold, read the slower stream until it catches up
    if (abs(diff) > thres) {												// if lag greater than 16.666
      if (diff < 0)															// lag < 0
        readframeto(t1, pipes[0], framenumbers[0], framesets[0]);			// read frame after specified timestamp (t1)
      else																	// lag >= 0
        readframeto(t0, pipes[1], framenumbers[1], framesets[1]);			// read frame after specified timestamp (t0)
      cerr << framenumbers[0] << "  " << framenumbers[1] << endl;			// error stream cam 1 and 2 frame arrival time
      continue; 
    }

    framesets[0] = align_to_color.process(framesets[0]);			// align cam 1 frame to color
    framesets[1] = align_to_color.process(framesets[1]);			// align cam 2 frame to color

    writeImages(framesets[0], dirs[0], count);						//
    writeImages(framesets[1], dirs[1], count);						//

    count++;														// increment frame count
  } while (1);


  end: 
  // step 3 end ///////////////////////////////////////////////////////////////////////
   // for (int j = 0; j < num_file; j++) {									// for 2 cameras
   // auto device = pipes[j]->get_active_profile().get_device();			// 
   // rs2::playback playback = device.as<rs2::playback>();				//
   // playback.set_real_time(true);										//
    
    // APPLICATION WOULD HANG DUE TO BUGS, manually stop use ctrl+c
    /*playback.stop(); */
    //pipes[j]->stop(); 
    // }
  system("pause");													// pause execution so can see output
  return 0;
}
