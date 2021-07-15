#include "../include/pcl_detector.h"
#include "../include/terminal.h"

// PCL viewer //
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

// Mutex: //
std::mutex cloud_mutex;

enum { COLS = 1280, ROWS = 720 };

struct callback_args {
  // structure used to pass arguments to the callback function
  PointCloudT::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

void cloud_cb_(const PointCloudT::ConstPtr &callback_cloud,
               PointCloudT::Ptr &cloud, bool *new_cloud_available_flag) {
  cloud_mutex.lock(); // for not overwriting the point cloud from another thread
  *cloud = *callback_cloud;
  *new_cloud_available_flag = true;
  cloud_mutex.unlock();
}

void pp_callback(const pcl::visualization::PointPickingEvent &event,
                 void *args) {
  struct callback_args *data = (struct callback_args *)args;
  if (event.getPointIndex() == -1)
    return;
  PointT current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->clicked_points_3d->points.push_back(current_point);
  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red(
      data->clicked_points_3d, 255, 0, 0);
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->clicked_points_3d, red,
                                 "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " "
            << current_point.z << std::endl;
}

void terminal_output(int people, float avg_framerate, float conf, float min_h,
                     float max_h, float vox_s, float closest) {
  std::string separator =
      "*******************************************************";
  std::string highlight = color(Term::fg::bright_green);
  std::string normal = color(Term::fg::bright_blue);
  std::string info = color(Term::fg::bright_yellow);
  std::string reset = color(Term::fg::reset);

  system("clear");
  std::cout << highlight << separator << std::endl;
  std::cout << "                     PCL Detector                      "
            << std::endl;
  std::cout << "                   Fresh Consulting                    "
            << std::endl;
  std::cout << separator << normal << std::endl;
  std::cout << std::endl;
  std::cout << "Number of people detected: " << info << people << normal
            << std::endl;
  std::cout << "Average framerate: " << info << avg_framerate << " Hz" << normal
            << std::endl;
  std::cout << std::endl;
  std::cout << "Minimum HOG confidence: " << info << conf << normal
            << std::endl;
  std::cout << "Minimum person height: " << info << min_h << normal
            << std::endl;
  std::cout << "Maximum person height: " << info << max_h << normal
            << std::endl;
  std::cout << "Voxel size: " << info << vox_s << normal << std::endl;
  std::cout << std::endl;
  std::cout << "Closest distance: " << info;
  if (closest == -1) {
    std::cout << "NaN";
  } else {
    std::cout << (closest * 3.28084) << " ft";
  }
  std::cout << normal << std::endl;
  std::string status = color(Term::fg::white) + "Clear" + reset;
  if (closest < 1 && closest >= 0) {
    status = color(Term::fg::red) + "Close" + reset;
  } else if (closest < 1.5 && closest > 0) {
    status = color(Term::fg::bright_magenta) + "Careful" + reset;
  } else if (closest < 2 && closest > 0) {
    status = color(Term::fg::yellow) + "Safe" + reset;
  } else if (closest < 20 && closest > 0) {
    status = highlight + "Detected" + reset;
  }
  std::cout << "Status: " << status << std::endl;
}

int detect(int argc, char **argv, pcl::Grabber *interface) {
  // Algorithm parameters:
  YAML::Node config = YAML::LoadFile("../config/people_detection_params.yaml");

  std::string svm_filename = config["svm"].as<std::string>();

  float min_confidence = config["conf"].as<float>();
  float min_height = config["min_h"].as<float>();
  float max_height = config["max_h"].as<float>();
  float min_width = config["min_w"].as<float>();
  float max_width = config["max_w"].as<float>();
  float voxel_size = config["vox_s"].as<float>();

  bool use_rgb_stream = config["use_rgb_stream"].as<bool>();

  // Set camera intrinsics
  Eigen::Matrix3f rgb_intrinsics_matrix;
  rgb_intrinsics_matrix << config["intrinsics"]["f_x"].as<float>(),
      config["intrinsics"]["s"].as<float>(),
      config["intrinsics"]["ppx"].as<float>(), 0.0,
      config["intrinsics"]["f_y"].as<float>(),
      config["intrinsics"]["ppy"].as<float>(), 0.0, 0.0, 1.0;

  // Read if some parameters are passed from command line:
  pcl::console::parse_argument(argc, argv, "--svm", svm_filename);
  pcl::console::parse_argument(argc, argv, "--conf", min_confidence);
  pcl::console::parse_argument(argc, argv, "--min_h", min_height);
  pcl::console::parse_argument(argc, argv, "--max_h", max_height);
  pcl::console::parse_argument(argc, argv, "--min_w", min_width);
  pcl::console::parse_argument(argc, argv, "--max_w", max_width);
  pcl::console::parse_argument(argc, argv, "--vox_s", voxel_size);

  // Set point cloud color:
  int r = config["color"]["r"].as<int>();
  int g = config["color"]["g"].as<int>();
  int b = config["color"]["b"].as<int>();

  // Read live stream:
  PointCloudT::Ptr cloud(new PointCloudT);
  bool new_cloud_available_flag = false;

  boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &)>
      f = [&](const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr
                  &callback_cloud) {
        cloud_cb_(callback_cloud, cloud, &new_cloud_available_flag);
      };
  interface->registerCallback(f);
  interface->start();

  // Wait for the first frame:
  while (!new_cloud_available_flag)
    std::this_thread::sleep_for(1ms);
  new_cloud_available_flag = false;

  cloud_mutex.lock(); // for not overwriting the point cloud

  // Display pointcloud:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(cloud, r, g, b);
  viewer.addPointCloud<PointT>(cloud, rgb, "input_cloud");
  viewer.setCameraPosition(0, 0, -2, 0, -1, 0, 0);

  // Add point picking callback to viewer:
  struct callback_args cb_args;
  PointCloudT::Ptr clicked_points_3d(new PointCloudT);
  cb_args.clicked_points_3d = clicked_points_3d;
  cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(&viewer);
  viewer.registerPointPickingCallback(pp_callback, (void *)&cb_args);
  std::cout << color(Term::fg::bright_blue)
            << "Shift+click on three floor points, then press 'Q'..."
            << color(Term::fg::reset) << std::endl;

  // Spin until 'Q' is pressed:
  viewer.spin();
  std::cout << color(Term::fg::bright_green) << "Complete"
            << color(Term::fg::reset) << std::endl;

  cloud_mutex.unlock();

  // Ground plane estimation:
  Eigen::VectorXf ground_coeffs;
  ground_coeffs.resize(4);
  std::vector<int> clicked_points_indices;
  for (unsigned int i = 0; i < clicked_points_3d->size(); i++)
    clicked_points_indices.push_back(i);
  pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
  model_plane.computeModelCoefficients(clicked_points_indices, ground_coeffs);
  std::cout << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1)
            << " " << ground_coeffs(2) << " " << ground_coeffs(3) << std::endl;

  // Initialize new viewer:
  pcl::visualization::PCLVisualizer viewer(
      "PCL Viewer"); // viewer initialization
  viewer.setCameraPosition(0, 0, -2, 0, -1, 0, 0);

  // Create classifier for people detection:
  pcl::people::PersonClassifier<pcl::RGB> person_classifier;
  person_classifier.loadSVMFromFile(svm_filename); // load trained SVM

  // People detection app initialization:
  pcl::people::GroundBasedPeopleDetectionApp<PointT>
      people_detector;                      // people detection object
  people_detector.setVoxelSize(voxel_size); // set the voxel size
  people_detector.setIntrinsics(
      rgb_intrinsics_matrix); // set RGB camera intrinsic parameters
  people_detector.setClassifier(person_classifier); // set person classifier
  people_detector.setPersonClusterLimits(min_height, max_height, min_width,
                                         max_width);

  // set sensor orientation to vertical by uncommenting the line below
  // people_detector.setSensorPortraitOrientation(true);

  // For timing:
  static unsigned count = 0;
  static double last = pcl::getTime();
  static float avg_framerate = 0.0;

  // Main loop:
  while (!viewer.wasStopped()) {
    if (new_cloud_available_flag &&
        cloud_mutex.try_lock()) // if a new cloud is available
    {
      new_cloud_available_flag = false;

      // Perform people detection on the new cloud:
      std::vector<pcl::people::PersonCluster<PointT>>
          clusters; // vector containing persons clusters
      people_detector.setInputCloud(cloud);
      people_detector.setGround(ground_coeffs); // set floor coefficients
      people_detector.compute(clusters);        // perform people detection

      ground_coeffs =
          people_detector.getGround(); // get updated floor coefficients

      // Draw cloud and people bounding boxes in the viewer:
      viewer.removeAllPointClouds();
      viewer.removeAllShapes();
      if (use_rgb_stream) {
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> camera(
            cloud);
        viewer.addPointCloud<PointT>(cloud, camera, "input_cloud");
      } else {
        pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(cloud, r,
                                                                     g, b);
        viewer.addPointCloud<PointT>(cloud, rgb, "input_cloud");
      }
      unsigned int k = 0;
      int closest = -1;
      for (std::vector<pcl::people::PersonCluster<PointT>>::iterator it =
               clusters.begin();
           it != clusters.end(); ++it) {
        if (it->getPersonConfidence() >
            min_confidence) // draw only people with confidence above a
                            // threshold
        {
          // draw theoretical person bounding box in the PCL viewer:
          it->drawTBoundingBox(viewer, k);
          k++;
          // update the distance between the sensor and closest cluster
          if (closest == -1 || closest > it->getDistance()) {
            closest = it->getDistance();
          }
        }
      }

      viewer.spinOnce();

      // Display average framerate:
      if (++count == 5) {
        double now = pcl::getTime();
        avg_framerate = double(count) / double(now - last);
        count = 0;
        last = now;
      }

      terminal_output(k, avg_framerate, min_confidence, min_height, max_height,
                      voxel_size, closest);

      cloud_mutex.unlock();
    }
  }
}