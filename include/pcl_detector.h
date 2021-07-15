#include "yaml-cpp/yaml.h"
#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>

#include <mutex>
#include <thread>

using namespace std::chrono_literals;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void cloud_cb_(const PointCloudT::ConstPtr &callback_cloud,
               PointCloudT::Ptr &cloud, bool *new_cloud_available_flag);

void pp_callback(const pcl::visualization::PointPickingEvent &event,
                 void *args);

int detect(int argc, char **argv, pcl::Grabber *interface);