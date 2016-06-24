#ifndef HECTOR_RAILS_DETECTION_H
#define HECTOR_RAILS_DETECTION_H

#include <ros/ros.h>


#include <cv.h>

#include <actionlib/server/simple_action_server.h>
#include <hector_perception_msgs/DetectObjectAction.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/grid_map_core.hpp>
#include <dynamic_reconfigure/server.h>
#include <hector_rails_detection/HectorRailsDetectionConfig.h>
#include <visualization_msgs/Marker.h>

namespace hector_rails_detection{

class RailsDetection{

public:
    RailsDetection();
    virtual ~RailsDetection();


    void elevationMapCallback(const grid_map_msgs::GridMap& grid_map_msg);
    void executeCallback(const hector_perception_msgs::DetectObjectGoalConstPtr& goal);



private:
    std::string type2str(int type) ;
    float computeRailSupportGeneric(const cv::Mat& img, int row, int col, float angle);
    float computeRailSupportMultiple(const cv::Mat& img, int row, int col, float angle);
    float computeRailSupport(const cv::Mat& img, int row, int col, float angle);

    void thresholdedDistance(const cv::Mat& img_in, cv::Mat& img_out);
    void computeRailSupport(const cv::Mat& img_in, cv::Mat& img_support, cv::Mat& img_max_orientation);
    void detectBlobs(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints);
    float computeBlobOrientation(const cv::Mat& img, cv::KeyPoint keypoint, float radius);
    void computeBlobOrientations(const cv::Mat& max_orientations, const std::vector<cv::KeyPoint>& keypoints, std::vector<float>& blob_orientations);
    void fitLineToBlob(const cv::Mat& max_orientations, const std::vector<cv::KeyPoint>& keypoints, const std::vector<float>& blob_orientations,  std::vector<std::pair<cv::Point2i,cv::Point2i>>& lines);
    int detectRails(cv::Mat& cv_img);


    ros::Subscriber elevation_map_subscriber_;
    ros::Publisher marker_publisher_;
    boost::shared_ptr<actionlib::SimpleActionServer<hector_perception_msgs::DetectObjectAction> > detection_object_server_;
    grid_map::GridMap grid_map_;
    float max_height_;
    float min_height_;


    //params
    double gradient_z_min_;
    double gradient_z_max_;
    double gradient_dif_min_;
    double gradient_dif_max_;
    double rail_support_feature_thresh_;
    double rail_support_feature_section_thresh_;

    //Dynamic reconfigure
    dynamic_reconfigure::Server<hector_rails_detection::HectorRailsDetectionConfig> dynamic_recf_server;
    dynamic_reconfigure::Server<hector_rails_detection::HectorRailsDetectionConfig>::CallbackType dynamic_recf_type;
    void dynamic_recf_cb(hector_rails_detection::HectorRailsDetectionConfig &config, uint32_t level);
    bool is_init;


};
}

#endif //HECTOR_RAILS_DETECTION_H
