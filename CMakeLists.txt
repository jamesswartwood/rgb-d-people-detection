set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(realsense_pcl_detector)
find_package(PCL 1.7 REQUIRED)
find_package(realsense2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} "./include")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCES src/pcl_detector.cpp src/real_sense_2_grabber.cpp include/real_sense_2_grabber.h include/pcl_detector.h)

# People detection executable with console argument options
add_executable (people_detection MACOSX_BUNDLE src/people_detection.cpp ${SOURCES})
target_link_libraries (people_detection ${PCL_LIBRARIES} ${realsense2_LIBRARY} /usr/local/lib/libyaml-cpp.a)

# Original demo executable # CONSIDER removing
add_executable (original_demo MACOSX_BUNDLE src/original_demo.cpp src/real_sense_2_grabber.cpp include/real_sense_2_grabber.h)
target_link_libraries (original_demo ${PCL_LIBRARIES} ${realsense2_LIBRARY} /usr/local/lib/libyaml-cpp.a)
