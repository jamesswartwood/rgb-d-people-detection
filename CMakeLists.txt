set(INCLUDE_FOLDER "./include")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(realsense_pcl_detector)
find_package(PCL 1.7 REQUIRED)
find_package(realsense2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${INCLUDE_FOLDER})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCES src/realsense_pcl_detector.cpp src/real_sense_2_grabber.cpp ${INCLUDE_FOLDER}/real_sense_2_grabber.h)
add_executable (realsense_pcl_detector MACOSX_BUNDLE ${SOURCES})
target_link_libraries (realsense_pcl_detector ${PCL_LIBRARIES})
target_link_libraries (realsense_pcl_detector ${realsense2_LIBRARY})