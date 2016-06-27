#ifndef Hector_Five_Pipes_Detection_H
#define Hector_Five_Pipes_Detection_H

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

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

#include <sensor_msgs/PointCloud2.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vigir_perception_msgs/PointCloudRegionRequest.h>

#include <dynamic_reconfigure/server.h>
#include <hector_five_pipes_detection/HectorFivePipesDetectionConfig.h>

#include <actionlib/server/simple_action_server.h>
#include <hector_perception_msgs/DetectObjectAction.h>

#include <hector_worldmodel_msgs/PosePercept.h>

namespace hector_five_pipes_detection{

class HectorFivePipesDetection{

public:
    HectorFivePipesDetection();
    virtual ~HectorFivePipesDetection();

protected:
    ros::Publisher orginal_pub_debug_;
    ros::Publisher after_pass_through_pub_debug_;
    ros::Publisher after_voxel_grid_pub_debug_;
    ros::Publisher final_cloud_pub_debug_;
    ros::Publisher plane_pub_debug_;
    ros::Publisher cloud_filtered_publisher_;
    ros::Publisher cluster_pub_debug_;
    ros::Publisher five_pipes_pos_pub_;
    ros::Publisher cluster_centers_pub_;
    ros::Publisher posePercept_pub_;

    ros::Subscriber pointcloud_sub_;
    ros::ServiceClient pointcloud_srv_client_;

    tf::TransformListener listener_;
    Eigen::Affine3d to_map_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
    void PclCallback(const sensor_msgs::PointCloud2 &pc_msg);

    void executeCallback(const hector_perception_msgs::DetectObjectGoalConstPtr& goal);

    bool findPipes(const geometry_msgs::Point& min, const geometry_msgs::Point& max, const std::string& frame_id);

private:
    //params
    double passThroughXMin_;
    double passThroughYMin_;
    double passThroughZMin_;
    double passThroughXMax_;
    double passThroughYMax_;
    double passThroughZMax_;
    double voxelGridX_;
    double voxelGridY_;
    double voxelGridZ_;
    double planeSegDistTresh_;
    double minRadius_;
    double maxRadius_;
    double clusterTolerance_;
    double searchRadius_;
    int minClusterSize_;
    int maxClusterSize_;
    int numberPointsThresh_;
    std::string worldFrame_;

    //Dynamic reconfigure
    dynamic_reconfigure::Server<hector_five_pipes_detection::HectorFivePipesDetectionConfig> dynamic_recf_server;
    dynamic_reconfigure::Server<hector_five_pipes_detection::HectorFivePipesDetectionConfig>::CallbackType dynamic_recf_type;
    void dynamic_recf_cb(hector_five_pipes_detection::HectorFivePipesDetectionConfig &config, uint32_t level);

    boost::shared_ptr<actionlib::SimpleActionServer<hector_perception_msgs::DetectObjectAction> > detection_object_server_;


};
}

#endif
