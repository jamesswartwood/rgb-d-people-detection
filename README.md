# RGB-D Real-time People Detection
People detection using RGB-D generated point clouds.

## People Detection Demo (GIF):
![Demo of the people detection](demo.gif)

## Required Packages
- librealsense SDK ([Installation Instructions](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md))
- yaml-cpp
  - Instructions:
    1. `git clone https://github.com/jbeder/yaml-cpp.git`
    2. `cd yaml-cpp`
    3. `mkdir build`
    4. `cd build`
    5. `cmake ..`
    6. `make`
    7. `sudo make install`


## Quick Start Guide
1. Clone this repository to your machine.
2. Within the main directory in your terminal, run these commands in order: `mkdir build`, `cd build`, `cmake ..`, `make`
3. Make sure that your RGB-D device is plugged into your computer
    - If you are using a RealSense D400 series camera, you can verify that it is working using `realsense-viewer`
4. Now run any of the executables located in your build folder using `./<name of file>`
    - Available executables:
      - `people_detection` *(main feature, recommended)*
      - `original_demo` *(original demo code from PCL documentation, adjusted for realsense)*
    - Available parameters:
      - `--help` or `-h`: display list of available parameters
      - `--device`: specify device (options: realsense *default*, openni, ti)
      - `--svm`: path to svm file
      - `--conf`: minimum HOG confidence
      - `--min_h`: minimum person height
      - `--max_h`: maximum person height
      - `--min_w`: minimum person width
      - `--max_w`: maximum person width
      - `--vox_s`: voxel size
    - Default values for the parameters above can be found and modified in the `config/people_detection_params.yaml` file.

## Setting Camera Intrinsics
1. In the `config` folder in the main directory, navigate to the `people_detection_params.yaml` file and edit the intrinsics values.
    - Explanations of each value can be found [here](https://ksimek.github.io/2013/08/13/intrinsic/)
    - You can find the intrinsics for any RealSense device using the [enumerate-devices](https://github.com/IntelRealSense/librealsense/tree/master/tools/enumerate-devices) tool

## Modifying Visual Display
- Point cloud:
  - This is also defined with RGB values in the `config/people_detection_params.yaml` file
- Bounding boxes around detected people:
  1. There is no easy way to do this except by altering the code in the PCL itself.
  2. In Visual Studio Code, go to line 251 in `pcl_detector.cpp` and hover your mouse over the `drawTBoundingBox` function. Right click this and select `Go to Definition`, which should navigate you to the function implementation in the `person_cluster.hpp` file in the PCL.
  3. Edit this however you wish. The changes will take place when you rebuild the executables.
      - Place `viewer.setRepresentationToWireframeForAllActors();` beneath line 388 to view the boxes as outlines rather than solid blocks.

## Velocity Tracking and Projection
- You can toggle velocity tracking and projection in the `config/people_detection_params.yaml` file. This feature will list all frame-by-frame estimated velocities in the terminal and will draw a line forward from each person's head representing their velocity in 3D space.
  - Because prior positions used in the velocity measurements are saved by cluster index, the program will have a hard time tracking multiple clusters separately when their indices might change between frames. 

## Known Issues
- The closest distance is only listed in multiples of a meter.
  - Next step: take a closer look at how the `getDistance` method on line 255 works

## Planned Features
#### Next Up
- Improved machine learning model
#### Stretch Goals
- Automated ground plane selection
- Support using a radar sensor instead of an RGB-D camera