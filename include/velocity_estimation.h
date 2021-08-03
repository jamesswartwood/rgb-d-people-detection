#include "../include/pcl_detector.h"
#include "../include/terminal.h"
#include <unordered_map>

using time_save =
    std::chrono::time_point<std::chrono::system_clock,
                            std::chrono::duration<double, std::ratio<1>>>;

class velocity_estimation {
private:
  time_save previous_time;
  std::unordered_map<int, Eigen::Vector3f> *people;

public:
  velocity_estimation(time_save current_time);
  double estimate(time_save current_time, int index, Eigen::Vector3f person,
                  pcl::visualization::PCLVisualizer &viewer);
  ~velocity_estimation();
};