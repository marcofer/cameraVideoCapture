cmake_minimum_required(VERSION 2.8)
project(cameraVideoCapture C CXX)

find_package( OpenCV 2.4.11 EXACT REQUIRED )
find_package(qibuild)

set(CMAKE_INSTALL_PREFIX /usr/local)
set(CMAKE_PREFIX_PATH /usr/local)

message(${qibuild_DIR})
message(${OpenCV_DIR})


SET (CMAKE_CXX_FLAGS                "-std=c++0x")

qi_create_bin(cameraVideoCapture "main.cpp")
qi_use_lib(cameraVideoCapture  ALCOMMON OPENCV2_CORE OPENCV2_HIGHGUI)
