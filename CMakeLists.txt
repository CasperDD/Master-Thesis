cmake_minimum_required(VERSION 3.0.0)
project(master VERSION 0.1.0)

include(CTest)
enable_testing()

set(raspicam_DIR "/usr/local/lib/cmake")

find_package( OpenCV REQUIRED)
find_package( Threads )
find_package(raspicam REQUIRED)
find_package(RPLidar)

include_directories( ${OpenCV_INCLUDE_DIRS} ${pigpio_INCLUDE_DIR} ${raspicam_CV_DIR})
add_executable(master main.cpp) 
TARGET_LINK_LIBRARIES(master raspicam pigpio ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} ${pigpio_LIBS} ${raspicam_CV_LIBS} RPLidar::RPLidar)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)