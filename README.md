------------------------- SOFTWARE -------------------------

 V  2 camera RGB/depth stream
 V  Map depth to color
 V  Sync between 2 pairs
 V  Cancel initial exposure
 V  Save RGB/depth frames as PNG
 V  Stream at stable 9fps (currently 640x480)
 V  Set fps, resolution
 V  Check single camera rgb/depth sync timestamp difference
 V  Raw bits, binary output
 X  Check single camera rgb/depth sync timestamp difference with faster fps
    Camera parameters (checkboard, matlab)
    - Get intrinsic parameters (Matlab), use single image/camera option
    - Calibrate all 4 cameras
    - Know which corresponds to which
    Output timestamp to text file, .csv file
    Save pointcloud
    Check align_to_color how it works
    Mex rectification integration

=====================================================================
========================= COMPLETE TUTORIAL ========================= 
=====================================================================

------------------------- Download SDK Examples  -------------------------
Get laptop
Download Visual Studio C++
Go to https://github.com/IntelRealSense/librealsense/releases
Go to Assets
Download latest Intel.RealSense.SDK-WIN10-2.24.0939.exe
Go to Local Disk (C:) -> Program Files (x86) -> IntelRealSense SDK 2.0
Open examples by clicking on ar-basic.vcxproj
Replace ar-basic.cpp code with own code
Right click project >> Set as Startup Project

------------------------- Install OpenCV with VCPKG -------------------------
Go to https://docs.microsoft.com/en-us/cpp/build/vcpkg?view=vs-2019
Download to C:/Program Files
Open cmd window as admin
cd C:/Program Files/vcpkg-master
bootstrap-vcpkg.bat
vcpkg search opencv
If machine x64 not x86: set VCPKG_DEFAULT_TRIPLET=x64-windows
vcpkg install opencv
vcpkg list
vcpkg integrate install (integrates vcpkg with Visual Studio)

------------------------- Add OpenCV to Visual Studio  -------------------------
Project >> align-save properties >> C/C++ >> General >> Additional Include Directories
  Add C:\Program Files\vcpkg-master\installed\x64-windows\include;

------------------------- Hardware  -------------------------
Attach camera sync cable
Plug left camera to left USB and right camera to right USB
Build code
Run Local Windows Debugger to start recording and streaming
Press stop debugging to stop recording and streaming

=====================================================================
=====================================================================
=====================================================================
