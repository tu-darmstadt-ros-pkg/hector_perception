#include <ros/ros.h>
#include <hector_five_pipes_detection/hector_five_pipes_detection.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "hector_five_pipes_detection_node");

  ROS_INFO("Starting HectorFivePipesDetection Node");
  hector_five_pipes_detection::HectorFivePipesDetection obj;
  ros::spin();
  exit(0);
}
