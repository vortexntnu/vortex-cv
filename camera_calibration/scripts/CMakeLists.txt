cmake_minimum_required(VERSION 3.10)
project(pls_work)

SET(OpenCV_DIR /home/vortex/opencv-344/installation/OpenCV-3.4.4/share/OpenCV/)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui calib3d imgcodecs)

include_directories(${OpenCV_INCLUDE_DIRS}) # not needed for opencv>=4.0
# add the executable
add_executable(pls_pls stereo_calib.cpp)
add_executable(pls_pls_pls camera_acquisition.cpp)

target_link_libraries(pls_pls ${OpenCV_LIBS})
target_link_libraries(pls_pls_pls ${OpenCV_LIBS})