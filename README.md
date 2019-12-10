multi-cam.cpp
- Timestamps, align, pointcloud, saving options
- Runs at properly at 6, 15, 30 fps
-	Made for streaming with two Intel RealSense cameras
-	Frames saved in fssLeft and fssRight vectors in memory
-	Commented out if(POINTCLOUD) and if(SAVE_PNG) to prevent fps lowering
-	Before running: enable SAVE_PNG to check frames saved in correct directories - if not invert CAM_SWITCHED

rs-align-save-v1.cpp
- Poll left camera, Process, Poll right camera, Process
- Real time
- Frames poorly synchronized

rs-align-save-v2.cpp	
- Poll left camera, Poll right camera, Process
- Real time
- Frames mostly synchronized

rs-save-to-disk.cpp 
- Extract .bag files, Process, Ensure synchronized timestamps  
- Post processing

## Software 

 V  2 camera RGB/depth stream  
 V  Map depth to color  
 V  Sync between 2 pairs  
 V  Cancel initial exposure  
 V  Save RGB/depth frames as PNG  
 V  Stream at stable fps (currently 640x480)  
 V  Allow user to set fps, resolution  
 V  Check single camera rgb/depth sync timestamp difference  
 V  Raw bits, binary output  
 V  Output timestamp to text file   
 V  Intrinsic parameters of 4 cameras (checkboard, matlab)  
 V  FPS for streaming (6, 15, 30, 60)  
 V  Check FPS after all processing  
 V  Save pointcloud  
 V  Master-Slave sync, disable auto RS2_OPTION_DISABLE_AUTO_EXPOSURE   
 V  Fix timestamp duplicates and skips  
 V  Ensure left and right cameras synchronized  
    
## Download SDK Examples
Get laptop  
Download Visual Studio C++  
Go to https://github.com/IntelRealSense/librealsense/releases  
Go to Assets  
Download latest Intel.RealSense.SDK-WIN10-2.24.0939.exe  
Go to Local Disk (C:) -> Program Files (x86) -> IntelRealSense SDK 2.0  
Open examples by clicking on multicam.vcxproj  
Replace rs-multicam.cpp code with own rs-multicam.cpp code  
Right click project >> Set as Startup Project  

## Install OpenCV with VCPKG 
Go to https://docs.microsoft.com/en-us/cpp/build/vcpkg?view=vs-2019  
Download to C:/Program Files  
Open cmd window as admin  
Type: cd C:/Program Files/vcpkg-master  
Type: bootstrap-vcpkg.bat  
Type: vcpkg search opencv  
If machine x64 not x86, type: set VCPKG_DEFAULT_TRIPLET=x64-window  
Type: vcpkg install opencv  
Type: vcpkg list  
Type: vcpkg integrate install (integrates vcpkg with Visual Studio)  

## Add OpenCV to Visual Studio
Project >> align-save properties >> C/C++ >> General >> Additional Include Directories  
  Add C:\Program Files\vcpkg-master\installed\x64-windows\include;  

## Hardware
Attach camera sync cable https://www.mouser.com/pdfdocs/Multiple_Camera_WhitePaper_rev11.pdf  
Plug left camera to left USB and right camera to right USB  
Build code
Run Local Windows Debugger to start recording and streaming  
Press stop debugging to stop recording and streaming  
