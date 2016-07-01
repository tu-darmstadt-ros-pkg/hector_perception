//author Benedikt Lückenrath, Johannes Schubert

#ifndef Hector_Five_Pipes_Detection_H
#define Hector_Five_Pipes_Detection_H

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <hector_worldmodel_msgs/PosePercept.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_representation.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vigir_perception_msgs/PointCloudRegionRequest.h>

#include <dynamic_reconfigure/server.h>
#include <hector_five_pipes_detection/HectorFivePipesDetectionConfig.h>

#include <actionlib/server/simple_action_server.h>
#include <hector_perception_msgs/DetectObjectAction.h>

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace hector_five_pipes_detection{

class HectorFivePipesDetection{

public:
    HectorFivePipesDetection();
    virtual ~HectorFivePipesDetection();

protected:
    ros::Publisher input_cloud_publisher;
    ros::Publisher filtered_cloud_debug_;
    ros::Publisher roi_debug_pub_;
    ros::Publisher cloud_without_planes_pub_debug_;
    ros::Publisher cluster_pub_debug_;
    ros::Publisher five_pipes_pos_pub_;
    ros::Publisher cluster_centers_pub_;
    ros::Publisher posePercept_pub_;
    ros::Publisher posePercept_debug_pub_;
    ros::Publisher endPoseDebugPCL_;


    ros::Subscriber realsense_pointcloud_sub_;
    ros::Subscriber LIDAR_pointcloud_sub_;
 //  ros::ServiceClient pointcloud_srv_client_;

 //   ros::Subscriber tf_sub_;
    bool robot_pose_init;
    Eigen::Quaternion<float> robot_rotation;
    Eigen::Vector3f robot_position;

    Eigen::Affine3d to_map_;

    tf::TransformListener tf_listener;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
    bool using_LIDAR;
    bool PCL_initiated_by_realsense;
    void PclCallback(const sensor_msgs::PointCloud2 &pc_msg);
    void LIDAR_PclCallback(const sensor_msgs::PointCloud2 &pc_msg);
  //  void TfCallback(const tf2_msgs::TFMessage &tf_msg);

    void executeCallback(const hector_perception_msgs::DetectObjectGoalConstPtr& goal);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cleanPointCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);

    bool findPipes(const geometry_msgs::Point& min, const geometry_msgs::Point& max, const std::string& frame_id);

private:
    //params
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
    float filter_radius_;
    int filter_cloud_n_neighbors_;
    float filter_cloud_max_stdev_;

    std::string worldFrame_;

    //Dynamic reconfigure
    dynamic_reconfigure::Server<hector_five_pipes_detection::HectorFivePipesDetectionConfig> dynamic_recf_server;
    dynamic_reconfigure::Server<hector_five_pipes_detection::HectorFivePipesDetectionConfig>::CallbackType dynamic_recf_type;
    void dynamic_recf_cb(hector_five_pipes_detection::HectorFivePipesDetectionConfig &config, uint32_t level);

    boost::shared_ptr<actionlib::SimpleActionServer<hector_perception_msgs::DetectObjectAction> > detection_object_server_;


};
}

#endif
