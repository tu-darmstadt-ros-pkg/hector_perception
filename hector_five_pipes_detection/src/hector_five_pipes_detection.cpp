#include <hector_five_pipes_detection/hector_five_pipes_detection.h>

#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>

namespace hector_five_pipes_detection{
HectorFivePipesDetection::HectorFivePipesDetection(){
  ROS_DEBUG ("HectorFivePipesDetection started");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");

  input_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  dynamic_recf_type = boost::bind(&HectorFivePipesDetection::dynamic_recf_cb, this, _1, _2);
  dynamic_recf_server.setCallback(dynamic_recf_type);

  cloud_sub_ = nh.subscribe("cloud", 10, &HectorFivePipesDetection::cloudCallback, this);

  input_cloud_pub_ = pnh.advertise<pcl::PointCloud<pcl::PointXYZ> >("input_cloud", 100, true);
  plane_seg_pub_ = pnh.advertise<pcl::PointCloud<pcl::PointXYZ> >("plane_segmentation", 100, true);
  depth_image_pub_ = pnh.advertise<sensor_msgs::Image>("depth_image", 100, true);
  center_point_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("center_point", 100, true);


  pose_percept_pub_ = nh.advertise<hector_worldmodel_msgs::PosePercept>("/worldmodel/pose_percept", 0);


  detection_object_server_.reset(
        new actionlib::SimpleActionServer<hector_perception_msgs::DetectObjectAction>(pnh, "detect", boost::bind(&HectorFivePipesDetection::executeCallback, this, _1) ,false));
  detection_object_server_->start();

  robot_pose_init = false;
}


void HectorFivePipesDetection::cloudCallback(const sensor_msgs::PointCloud2& pc_msg){
  pcl::fromROSMsg(pc_msg, *input_cloud_);
}

void HectorFivePipesDetection::executeCallback(const hector_perception_msgs::DetectObjectGoalConstPtr& goal)
{
  hector_perception_msgs::DetectObjectResult result;
  result.detection_success = findPipes(goal->detect_request.roi_hint.bounding_box_min, goal->detect_request.roi_hint.bounding_box_max);
  detection_object_server_->setSucceeded(result);
}

bool HectorFivePipesDetection::intersectPlane(const Eigen::Vector3d &n, const Eigen::Vector3d &p0, const Eigen::Vector3d &l0, const Eigen::Vector3d &l, double &t)
{
    // assuming vectors are all normalized
    double denom = n.dot(l);
    if (denom > 1e-6) {
        Eigen::Vector3d p0l0 = p0 - l0;
        t = p0l0.dot(n) / denom;
        return (t >= 0);
    }
    return false;
}

bool HectorFivePipesDetection::findPipes(const geometry_msgs::Point& min, const geometry_msgs::Point& max)
{
  ROS_INFO_STREAM("Running detection");
  if (input_cloud_->empty()){
      ROS_WARN("Input cloud is empty");
      return false;
  }
  input_cloud_pub_.publish(input_cloud_); // to be able to check



  std::vector<int> mapping;
  pcl::removeNaNFromPointCloud(*input_cloud_, *input_cloud_, mapping);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.03);
  seg.setAxis(Eigen::Vector3f(0, 0, 1)); // search along optical axis
  seg.setEpsAngle(M_PI/4);
  seg.setMaxIterations(1000);

  seg.setInputCloud(input_cloud_);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    ROS_ERROR("Could not estimate a planar model for the given dataset.");
    return false;
  }

  // Extract plane inliers
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(input_cloud_);
  extract.setIndices(inliers);

  pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>());
  extract.filter(*plane);
  plane_seg_pub_.publish(plane);

  double nx = coefficients->values[0]; double ny = coefficients->values[1]; double nz = coefficients->values[2];
  pcl::CentroidPoint<pcl::PointXYZ> centroid;
  for (unsigned int i = 0; i < plane->size(); i++) {
    const pcl::PointCloud<pcl::PointXYZ> & plane_ref = *(plane);
    centroid.add(plane_ref[i]);
  }
  pcl::PointXYZ centroid_point;
  centroid.get(centroid_point);
  Eigen::Vector3d plane_origin(centroid_point.x, centroid_point.y, centroid_point.z);
  Eigen::Vector3d plane_normal(nx, ny, nz);
  ROS_INFO_STREAM("Plane normal: " << nx << ", " << ny << ", " << nz);

  pcl::RangeImagePlanar range_image;
  cv::Mat depth_image;
  pcl_to_cv_proc::generateDepthImage(*plane, Eigen::Affine3d::Identity(), range_image, depth_image);
  depth_image.convertTo(depth_image, CV_8UC1);
  cv::imwrite("depth_image.png", depth_image);

  cv::Mat blurred;
  cv::blur(depth_image, blurred, cv::Size(7, 7));

  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(blurred, circles, CV_HOUGH_GRADIENT, 1, 100, 50, 30, 0, 0);
  ROS_INFO_STREAM("Found " << circles.size() << " circles.");

  cv::Mat circle_image;
  cv::cvtColor(blurred, circle_image, CV_GRAY2RGB);

  float min_dist = std::numeric_limits<float>::max();
  int center_index = -1;
  for( size_t i = 0; i < circles.size(); i++ )
  {
    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);
    // circle center
    cv::circle( circle_image, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
    // circle outline
    cv::circle( circle_image, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );

    float dist = 0;
    for (size_t j = 0; j < circles.size(); j++) {
      cv::Vec3f diff = circles[i] - circles[j];
      dist += diff.dot(diff);
    }
    ROS_INFO_STREAM("Dist: " << dist);
    if (dist < min_dist) {
      min_dist = dist;
      center_index = i;
    }
  }
  ROS_INFO_STREAM("Center circle: " << circles[center_index]);

  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = ros::Time::now();
  cv_image.header.frame_id = input_cloud_->header.frame_id;
  cv_image.encoding = sensor_msgs::image_encodings::RGB8;
  cv_image.image = circle_image;

  depth_image_pub_.publish(cv_image.toImageMsg());

  sensor_msgs::CameraInfo camera_info;
  camera_info.width = 600;
  camera_info.height = camera_info.width;
  camera_info.K[0] = camera_info.width;
  camera_info.K[4] = camera_info.K[0];
  camera_info.P[0] = camera_info.K[0];
  camera_info.P[5] = camera_info.K[4];

  camera_info.K[2] = camera_info.width / 2;
  camera_info.K[5] = camera_info.height / 2;
  camera_info.P[2] = camera_info.K[2];
  camera_info.P[6] = camera_info.K[5];

  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(camera_info);
  for (size_t i = 0; i < circles.size(); i++) {
    cv::Point2d center(circles[i][0], circles[i][1]);
    ROS_INFO_STREAM("Center: " << center.x << ", " << center.y);
    cv::Point3d ray = model.projectPixelTo3dRay(center);
    ROS_INFO_STREAM("ray: " << ray.x << ", " << ray.y << ", " << ray.z);

    double t;
    Eigen::Vector3d ray_eigen(ray.x, ray.y, ray.z);
    if (intersectPlane(plane_normal, plane_origin, Eigen::Vector3d::Zero(), ray_eigen, t)) {
      Eigen::Vector3d point = t * ray_eigen;
      ROS_INFO_STREAM("Point: " << point(0) << ", " << point(1) << ", " << point(2));

      if (i == center_index) {
        geometry_msgs::PoseStamped pose_msg;
        tf::pointEigenToMsg(point, pose_msg.pose.position);
        pose_msg.header.frame_id = input_cloud_->header.frame_id;
//        pose_msg.header.stamp = input_cloud_->header.stamp;
        center_point_pub_.publish(pose_msg);
      }
    } else {
      ROS_WARN_STREAM("Couldn't intersect point");
    }
  }





//    if (success){
//        hector_worldmodel_msgs::PosePercept pp;
//        pp.header.frame_id= start_check_positions->header.frame_id;
//        pp.header.stamp= ros::Time(0);
//        pp.info.class_id= "pipes";
//        pp.info.class_support=1;
//        pp.info.object_support=1;
//        pp.pose.pose.position.x= centerPoint.x;
//        pp.pose.pose.position.y= centerPoint.y;
//        pp.pose.pose.position.z= centerPoint.z;
//        pp.pose.pose.orientation.x= quat.x();
//        pp.pose.pose.orientation.y= quat.y();
//        pp.pose.pose.orientation.z= quat.z();
//        pp.pose.pose.orientation.w= quat.w();
//        std::vector <float>flatpointarray; // watch out for changing size of n pipes!!
//        for (int i = 0; i < sortedListOfCenters.size(); i++){
//            pcl::PointXYZ p = sortedListOfCenters[i];
//            flatpointarray.push_back(p.x);
//            flatpointarray.push_back(p.y);
//            flatpointarray.push_back(p.z);
//        }
//        pp.info.data = flatpointarray;

//        geometry_msgs::PoseStamped pose_debug;
//        pose_debug.header.frame_id = pp.header.frame_id;
//        pose_debug.header.stamp = pp.header.stamp;
//        pose_debug.pose.orientation.w = quat.w();
//        pose_debug.pose.orientation.x = quat.x();
//        pose_debug.pose.orientation.y = quat.y();
//        pose_debug.pose.orientation.z = quat.z();
//        pose_debug.pose.position.x = centerPoint.x;
//        pose_debug.pose.position.y = centerPoint.y;
//        pose_debug.pose.position.z = centerPoint.z;
//        posePercept_pub_.publish(pp);
//        posePercept_debug_pub_.publish(pose_debug);
//        ROS_INFO("PosePercept startcheck postion pipes published");
//    }


  return true;
}

void HectorFivePipesDetection::dynamic_recf_cb(hector_five_pipes_detection::HectorFivePipesDetectionConfig &config, uint32_t level)
{
  priorityRGBD_= config.priorityRGBD;
  priorityLIDAR_= config.priorityLIDAR;
  x_min_dist_BB_= config.x_min_dist_BB;
  x_max_dist_BB_= config.x_max_dist_BB;
  y_tolarance_BB_= config.y_tolarance_BB;
  z_min_dist_BB_= config.z_min_dist_BB;
  z_max_dist_BB_= config.z_max_dist_BB;
  planeSegDistTresh_= config.planeSegDistTresh;
  numberPointsThresh_= config.numberPointsThresh;
  clusterTolerance_= config.clusterTolerance;
  minClusterSize_= config.minClusterSize;
  maxClusterSize_= config.maxClusterSize;
  searchRadius_= config.searchRadius;
  do_min_cluster_radius_ = config.do_min_cluster_radius;
  min_cluster_radius_ = config.min_cluster_radius;
  doFilterCloud_pre_plane_= config.doFilterCloud_pre_plane;
  filter_cloud_n_neighbors_pre_plane_ = config.filter_cloud_n_neighbors_pre_plane;
  filter_cloud_max_stdev_pre_plane_ = config.filter_cloud_max_stdev_pre_plane;
  doFilterCloud_post_plane_= config.doFilterCloud_post_plane;
  filter_cloud_n_neighbors_post_plane_ = config.filter_cloud_n_neighbors_post_plane;
  filter_cloud_max_stdev_post_plane_ = config.filter_cloud_max_stdev_post_plane;
}

}


