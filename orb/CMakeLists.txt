cmake_minimum_required(VERSION 2.8)

project(test)

#add_library(hello_shared SHARED libHelloSLAM.cpp)
INCLUDE_DIRECTORIES(
"/usr/local/include"
)

link_directories(
"/usr/local/lib"
)

link_libraries(
opencv_core
opencv_calib3d
opencv_imgproc
opencv_features2d
opencv_imgcodecs
opencv_highgui
opencv_videoio
)

add_executable(test test.cpp)
add_executable(test_webcam test_webcam.cpp)
