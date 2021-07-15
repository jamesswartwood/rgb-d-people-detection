#include "../include/pcl_detector.h"
#include "../include/real_sense_2_grabber.h"
#include <iostream>
#include <pcl/console/parse.h>

int print_help() {
  std::cout << "*******************************************************"
            << std::endl;
  std::cout << "Ground based people detection app options:" << std::endl;
  std::cout << "   --help    <show_this_help>" << std::endl;
  std::cout << "   --device  <sensor to use: realsense (default), openni, or "
               "ti>"
            << std::endl;
  std::cout << "   --svm     <path_to_svm_file>" << std::endl;
  std::cout << "   --conf    <minimum_HOG_confidence>" << std::endl;
  std::cout << "   --min_h   <minimum_person_height>" << std::endl;
  std::cout << "   --max_h   <maximum_person_height>" << std::endl;
  std::cout << "   --min_w   <minimum_person_width>" << std::endl;
  std::cout << "   --max_w   <maximum_person_width>" << std::endl;
  std::cout << "   --vox_s   <voxel_size>" << std::endl;
  std::cout << "*******************************************************"
            << std::endl;
  return 0;
}

int main(int argc, char **argv) {
  if (pcl::console::find_switch(argc, argv, "--help") ||
      pcl::console::find_switch(argc, argv, "-h"))
    return print_help();

  // Set default device to be a RealSense camera
  std::string device = "realsense";

  // Set the device to whatever was passed in as an argument
  pcl::console::parse_argument(argc, argv, "--device", device);

  // Instantiate the interface grabber
  pcl::Grabber *interface;
  if (device == "realsense") {
    interface = new pcl::RealSense2Grabber();
  } else if (device == "openni") {
    interface = new pcl::OpenNIGrabber();
  } else if (device == "ti") {
    std::cout << "INFO: This sensor is not supported yet" << std::endl;
    return 0;
  } else {
    std::cout << "INFO: Unknown device. Use --help for more information."
              << std::endl;
    return 0;
  }

  // Run the detection program
  return detect(argc, argv, interface);
}