#include <ros/ros.h>
#include <hector_rails_detection/rails_detection.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "hector_rails_detection_node");

  ROS_INFO("Starting HectorRailsDetection Node");
  hector_rails_detection::RailsDetection obj;
  ros::spin();
  exit(0);
}
