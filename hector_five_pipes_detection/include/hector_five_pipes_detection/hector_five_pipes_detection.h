#ifndef Hector_Five_Pipes_Detection_H
#define Hector_Five_Pipes_Detection_H

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <hector_worldmodel_msgs/PosePercept.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <dynamic_reconfigure/server.h>
#include <hector_five_pipes_detection/HectorFivePipesDetectionConfig.h>

#include <actionlib/server/simple_action_server.h>
#include <hector_perception_msgs/DetectObjectAction.h>

#include <Eigen/Eigen>
#include <pcl_to_cv_proc/to_cv_depth_img.h>

#include <opencv2/imgcodecs.hpp>

#include <image_geometry/pinhole_camera_model.h>


namespace hector_five_pipes_detection{

class HectorFivePipesDetection{

public:
  HectorFivePipesDetection();
protected:
  ros::Publisher input_cloud_pub_;
  ros::Publisher plane_seg_pub_;
  ros::Publisher pose_percept_pub_;
  ros::Publisher center_point_pub_;

  ros::Publisher depth_image_pub_;

  ros::Subscriber cloud_sub_;

  bool robot_pose_init;
  Eigen::Quaternion<float> robot_rotation;
  Eigen::Vector3f robot_position;

  Eigen::Affine3d to_map_;

  tf::TransformListener tf_listener;

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;
  void cloudCallback(const sensor_msgs::PointCloud2 &pc_msg);

  void executeCallback(const hector_perception_msgs::DetectObjectGoalConstPtr& goal);

  bool findPipes(const geometry_msgs::Point& min, const geometry_msgs::Point& max);
  bool intersectPlane(const Eigen::Vector3d &n, const Eigen::Vector3d &p0, const Eigen::Vector3d &l0, const Eigen::Vector3d &l, double &t);

private:
  //params
  float priorityRGBD_;
  float priorityLIDAR_;
  float x_min_dist_BB_;
  float x_max_dist_BB_;
  float y_tolarance_BB_;
  float z_min_dist_BB_;
  float z_max_dist_BB_;
  float planeSegDistTresh_;
  int numberPointsThresh_;
  float clusterTolerance_;
  int minClusterSize_;
  int maxClusterSize_;
  float searchRadius_;
  float do_min_cluster_radius_;
  float min_cluster_radius_;
  float doFilterCloud_pre_plane_;
  int filter_cloud_n_neighbors_pre_plane_;
  float filter_cloud_max_stdev_pre_plane_;
  float doFilterCloud_post_plane_;
  int filter_cloud_n_neighbors_post_plane_;
  float filter_cloud_max_stdev_post_plane_;

  std::string world_frame_;

  //Dynamic reconfigure
  dynamic_reconfigure::Server<hector_five_pipes_detection::HectorFivePipesDetectionConfig> dynamic_recf_server;
  dynamic_reconfigure::Server<hector_five_pipes_detection::HectorFivePipesDetectionConfig>::CallbackType dynamic_recf_type;
  void dynamic_recf_cb(hector_five_pipes_detection::HectorFivePipesDetectionConfig &config, uint32_t level);

  boost::shared_ptr<actionlib::SimpleActionServer<hector_perception_msgs::DetectObjectAction> > detection_object_server_;


};
}

#endif
