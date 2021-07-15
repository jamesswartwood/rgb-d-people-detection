# rgb-d-people-detection
Repository for people detection script through Realsense D455.

## Required Packages:
- librealsense SDK ([Installation Instructions](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md))

## Quick Start Guide:
1. Clone this repository to your machine.
2. Within the main directory in your terminal, run these commands in order: `mkdir build`, `cd build`, `cmake ..`, `make`
3. Make sure that your RealSense D400 series camera is plugged into your computer and working in `realsense-viewer`
4. Now run the program using `./rgb-d-people-detection`, which is located in your build folder

## Known Issues:
- The infrared stream of the RealSense camera is not configured
  - Temporarily commenting out the line that checks the format of the stream
  - Next step: find what could be causing the error in the stream
- The ground selection visualization does not do well in displaying the point cloud
  - The points are being displayed too small
  - Next step: figure out how to increase their size
- As of yet, the executable has not been able to recognize any humans
  - Proven to work with OpenNI ([preview](https://pcl.readthedocs.io/projects/tutorials/en/latest/_images/Screen8.jpg), [source](https://pcl.readthedocs.io/projects/tutorials/en/latest/ground_based_rgbd_people_detection.html))
  - Next step: look into how the RS point cloud data might different from the ONI point cloud data