#include "../include/velocity_estimation.h"

velocity_estimation::velocity_estimation(time_save current_time) {
  previous_time = current_time;
  people = new std::unordered_map<int, Eigen::Vector3f>();
}

double
velocity_estimation::estimate(time_save current_time, int index,
                              Eigen::Vector3f person,
                              pcl::visualization::PCLVisualizer &viewer) {
  // Check for the case where the index is not yet in the unordered map
  if (people->find(index) == people->end()) {
    previous_time = current_time;
    people->insert({index, person});
    return 0;
  }

  // Find the person's old coordinates
  Eigen::Vector3f old_coords = people->at(index);

  // Find the change in position
  Eigen::Vector3f difference = person - old_coords;

  // Convert to distance representation
  double distance =
      (difference - (difference.dot(old_coords / old_coords.norm()) *
                     (old_coords / old_coords.norm())))
          .norm();

  // Find the velocity
  double velocity = distance / (current_time - previous_time).count();

  if (velocity < 0.5) {
    pcl::PointXYZ front =
        pcl::PointXYZ(2 * (person.x() - old_coords.x()) + person.x(),
                      2 * (person.y() - old_coords.y()) + person.y(),
                      2 * (person.z() - old_coords.z()) + person.z());
    pcl::PointXYZ back = pcl::PointXYZ(person.x(), person.y(), person.z());

    std::string arrow_name = "arrow_" + index;
    viewer.addLine(back, front, 0.4f, 0.8f, 1.0f, arrow_name);
    viewer.setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, arrow_name);
  }

  // Save values
  people->erase(index);
  people->insert({index, person});
  previous_time = current_time;

  // Return estimated velocity
  return velocity;
}

velocity_estimation::~velocity_estimation() { delete people; }
