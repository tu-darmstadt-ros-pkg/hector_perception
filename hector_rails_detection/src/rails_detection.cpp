//=================================================================================================
// Copyright (c) 2016, Kevin Daun, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo/photo.hpp>
#include <eigen3/Eigen/Core>

#include <hector_rails_detection/rails_detection.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/grid_map_core.hpp>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <hector_worldmodel_msgs/PosePercept.h>

#include "opencv2/features2d.hpp"

namespace hector_rails_detection
{
RailsDetection::RailsDetection()
{
    ros::NodeHandle nh("");
    elevation_map_subscriber_ = nh.subscribe("/elevation_mapping/elevation_map_global",10, &RailsDetection::elevationMapCallback, this);
    nh.param("track_length", track_length_, 2.4);
    nh.param("track_width", track_width_, 0.5);
    nh.param("rail_width", rail_width_, 0.1);
    nh.param("n_rails", n_rails_, 2);
    nh.param("gradient_z_min", gradient_z_min_, -0.5);
    nh.param("gradient_z_max", gradient_z_max_, 2.0);
    nh.param("gradient_dif_min", gradient_dif_min_, 0.0);
    nh.param("gradient_dif_max", gradient_dif_max_, 2.0);
    nh.param("rail_support_feature_thresh", rail_support_feature_thresh_, 1.3);
    nh.param("rail_support_feature_section_thresh", rail_support_feature_section_thresh_, 0.3);

    marker_publisher_ = nh.advertise<visualization_msgs::Marker>("hector_rail_detection/line_perecepts", 0);
    pose_percept_publisher_= nh.advertise<hector_worldmodel_msgs::PosePercept>("/worldmodel/pose_percept", 0);
    is_init = false;
    ros::NodeHandle pnh("~");
    detection_object_server_.reset(new actionlib::SimpleActionServer<hector_perception_msgs::DetectObjectAction>(pnh, "detect", boost::bind(&RailsDetection::executeCallback, this, _1) ,false));
    detection_object_server_->start();
    dynamic_recf_type = boost::bind(&RailsDetection::dynamic_recf_cb, this, _1, _2);
    dynamic_recf_server.setCallback(dynamic_recf_type);
}
RailsDetection::~RailsDetection()
{}


void RailsDetection::elevationMapCallback(const grid_map_msgs::GridMap& grid_map_msg)
{
    std::vector<float> data0 = grid_map_msg.data[0].data;

    int p = 0;
    max_height_ = std::numeric_limits<float>::lowest();
    min_height_ = std::numeric_limits<float>::infinity();

    for (auto &val : data0)
    {
        if (!std::isnan(val) && val>max_height_)
            max_height_ = val;
        if (!std::isnan(val) && val<min_height_)
            min_height_ = val;
        p++;
    }

    grid_map::GridMapRosConverter::fromMessage(grid_map_msg, grid_map_);
    is_init = true;
}

void RailsDetection::executeCallback(const hector_perception_msgs::DetectObjectGoalConstPtr& goal)
{
    ROS_INFO("callback");

    hector_perception_msgs::DetectObjectResult result;
    cv::Mat image;


    int encoding = CV_32F;
    std::string layer = "elevation";

    if (grid_map_.getSize()(0) > 0 && grid_map_.getSize()(1) > 0) {
        image = cv::Mat::zeros(grid_map_.getSize()(0), grid_map_.getSize()(1), encoding);
    } else {
        std::cerr << "Invalid grid map?" << std::endl;
    }
    float lowerValue = min_height_;
    float upperValue = max_height_;

    // Clamp outliers.
    grid_map::GridMap map = grid_map_;
    map.get(layer) = map.get(layer).unaryExpr(grid_map::Clamp<float>(lowerValue, upperValue));
    const grid_map::Matrix& data = grid_map_[layer];

    // Convert to image.
    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);
        if (std::isfinite(data(index(0), index(1)))) {
            const float& value = data(index(0), index(1));
            const float imageValue = (float) value;//(value - lowerValue) / (upperValue - lowerValue)); //* (float) imageMax);
            const grid_map::Index imageIndex(iterator.getUnwrappedIndex());
            unsigned int channel = 0;
            image.at<cv::Vec<float, 1>>(imageIndex(0), imageIndex(1))[channel] = imageValue;
        }
    }

    detectRails(image);
    detection_object_server_->setSucceeded(result);
}



typedef std::vector<float> section;

std::string RailsDetection::type2str(int type) {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}

float RailsDetection::computeRailSupportGeneric(const cv::Mat& img, int row, int col, float angle, float resolution)
{
    std::vector<section> sections;
    float r = resolution;
/*
    sections.push_back(std::vector<float>{-0.45f/r,-0.25f/r,-1.0});
    sections.push_back(std::vector<float>{-0.35f/r,-0.15f/r,1.0});
    sections.push_back(std::vector<float>{-0.25/r,0.25/r,-1.0});
    sections.push_back(std::vector<float>{0.15f/r,0.35f/r,1.0});
    sections.push_back(std::vector<float>{0.25f/r,0.45f/r,-1.0});
    */
    float d0 = track_width_*0.5 - rail_width_*0.5;
    float d1 = track_width_*0.5 + rail_width_*0.5;
    float d2 = track_width_*0.5 + rail_width_*1.0;

    // Detectects gradient pattern: _-___-_
    sections.push_back(std::vector<float>{-(d2)/r,-(d1)/r,-1.0});
    sections.push_back(std::vector<float>{-(d1)/r,-(d0)/r,1.0});
    sections.push_back(std::vector<float>{-(d0)/r,(d0)/r,-1.0});
    sections.push_back(std::vector<float>{(d0)/r,(d1)/r,1.0});
    sections.push_back(std::vector<float>{(d1)/r,(d2)/r,-1.0});


    float support = 0.;

    for(section sec : sections)
    {
        float dx0 = cos(angle)*sec[0];
        float dy0 = sin(angle)*sec[0];
        float dx1 = cos(angle)*sec[1];
        float dy1 = sin(angle)*sec[1];
        float x0 = col + dx0;
        float y0 = row + dy0;
        float x1 = col + dx1;
        float y1 = row + dy1;
        cv::LineIterator line_it(img, cv::Point2f(x0,y0),cv::Point2f(x1,y1));
        float pos = 0.;
        float neg = 0.;
        for(int i_it = 0; i_it<line_it.count; i_it++)
        {
            float weight = 1.;//std::abs((float)i_it-0.5*(float)line_it.count)/(float)line_it.count; //reduced weight for elements in the middle of the section
            if(sec[2]*(img.at<float>(line_it.pos())-0.1) > 0.0)
            {
                pos += weight;
            }
            else
            {
                neg += weight;
            }
            line_it++;
        }
        if(pos/(neg+pos)>rail_support_feature_section_thresh_)
            support++;
    }
    return support;
}

float RailsDetection::computeRailSupportMultiple(const cv::Mat& img, int row, int col, float angle = 0.0, float resolution)
{
    float support0 = computeRailSupportGeneric(img, row, col, angle, resolution);

    int dx0 = round(cos(angle+M_PI_2)*1.4);
    int dy0 = round(sin(angle+M_PI_2)*1.4);

    float support1 = computeRailSupportGeneric(img, row+dy0, col+dx0, angle, resolution);
    float support2 = computeRailSupportGeneric(img, row-dy0, col-dx0, angle, resolution);

    return (2.0*support0+support1+support2)/4.0;


}

void RailsDetection::thresholdedDistance(const cv::Mat& img_in, cv::Mat& img_out)
{

    img_in.copyTo(img_out);
    img_out.setTo(0);

    ROS_INFO("params %f %f %f %f",gradient_z_min_,gradient_z_max_,gradient_dif_min_,gradient_dif_max_);
    for(unsigned int i_r = 5; i_r < img_in.rows-5; ++i_r)
    {
        for(unsigned int i_c = 5; i_c < img_in.cols-5; ++i_c)
        {
            for(int i_x = -2; i_x < 3; ++i_x)
            {
                for(int i_y = -2; i_y < 3; ++i_y)
                {
                    if(!(i_x ==0 && i_y == 0))
                    {
                        float delta_temp = (img_in.at<float>(i_r,i_c)-img_in.at<float>(i_r+i_x,i_c+i_y));
                        bool valid_height =  img_in.at<float>(i_r,i_c)> gradient_z_min_ &&  img_in.at<float>(i_r,i_c)< gradient_z_max_ &&img_in.at<float>(i_r+i_x,i_c+i_y)> gradient_z_min_ &&  img_in.at<float>(i_r+i_x,i_c+i_y)< gradient_z_max_;
                        bool valid_grad =  delta_temp < 5*gradient_dif_max_ && delta_temp > gradient_dif_min_;
                        float delta = 0.0;
                        if (valid_height && valid_grad)
                        {
                            delta = delta_temp;
                            if(delta>(gradient_dif_max_-gradient_dif_min_))
                                delta = gradient_dif_max_;

                        }
                        if(!valid_height)
                            delta -= 100;
                        img_out.at<float>(i_r,i_c) += delta;
                    }
                }
            }

            if(img_out.at<float>(i_r,i_c) < 0 )
                img_out.at<float>(i_r,i_c) = 0;
        }
    }
}

void RailsDetection::computeRailSupport(const cv::Mat& img_in, cv::Mat& img_support, cv::Mat& img_max_orientation)
{
    img_in.copyTo(img_support);
    img_support.setTo(0);
    img_support.copyTo(img_max_orientation);
    // std::cout<<img_in<<std::endl;
    ROS_INFO("Thrsh is: %f",rail_support_feature_thresh_);
    for(unsigned int i_r = 5; i_r < img_in.rows-5; ++i_r)
    {
        for(unsigned int i_c = 5; i_c < img_in.cols-5; ++i_c)
        {
            std::vector<float> supports;
            for(float i_orient = 0.; i_orient<8.; i_orient++)
                supports.push_back(computeRailSupportMultiple(img_in,i_r,i_c,M_PI*i_orient/8.0));
            auto support_it = std::max_element(supports.begin(),supports.end());
            int max_orient = std::distance(supports.begin(), support_it);
            float support = *support_it;

            if(support >= rail_support_feature_thresh_)
            {
                img_support.at<float>(i_r,i_c) =(1.0/(pow(5.0,2)))*support*support;
                img_max_orientation.at<float>(i_r,i_c) = M_PI*(float)max_orient/8.;
            }
            else
            {
                img_max_orientation.at<float>(i_r,i_c) = -1.;
            }
        }
    }
}


void RailsDetection::detectBlobs(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints)
{

    cv::Mat blob_base;
    img.convertTo(blob_base,CV_8UC1);
    cv::SimpleBlobDetector::Params params;

    // Change thresholds
    params.minThreshold = 10;
    params.maxThreshold = 2000;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 100;
    params.maxArea = 10000;

    // Filter by Circularity
    params.filterByCircularity = false;
    params.minCircularity = 0.1;

    // Filter by Convexity
    params.filterByConvexity = false;
    params.minConvexity = 0.087;

    // Filter by Inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0.01;
    cv::SimpleBlobDetector detector;
    detector.create(params);

    params.filterByColor = false;

    // Detect blobs.
    keypoints.clear();
    detector.detect( blob_base, keypoints);
}


double getOrientation(const std::vector<cv::Point> &pts)
{
    //Construct a buffer used by the pca analysis
    int sz = static_cast<int>(pts.size());
    cv::Mat data_pts = cv::Mat(sz, 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }
    //Perform PCA analysis
    cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);
    //Store the center of the object
    cv::Point cntr = cv::Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                               static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
    //Store the eigenvalues and eigenvectors
    std::vector<cv::Point2d> eigen_vecs(2);
    std::vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                    pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }
    /*
    // Draw the principal components
    circle(img, cntr, 3, Scalar(255, 0, 255), 2);
    Point p1 = cntr + 0.02 * Point(static_cast<int>(eigen_vecs[0].x * eigen_val[0]), static_cast<int>(eigen_vecs[0].y * eigen_val[0]));
    Point p2 = cntr - 0.02 * Point(static_cast<int>(eigen_vecs[1].x * eigen_val[1]), static_cast<int>(eigen_vecs[1].y * eigen_val[1]));
    drawAxis(img, cntr, p1, Scalar(0, 255, 0), 1);
    drawAxis(img, cntr, p2, Scalar(255, 255, 0), 5);*/
    double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
    return angle;
}

float RailsDetection::computeBlobOrientationPCA(const cv::Mat& img, cv::KeyPoint keypoint, float radius)
{


    std::vector<cv::Point> points;
    for(int i_x = keypoint.pt.x -radius; i_x <keypoint.pt.x + radius; i_x++)
    {
        for(int i_y = keypoint.pt.y -radius; i_y <keypoint.pt.y + radius; i_y++)
        {
            float dx = i_x -keypoint.pt.x;
            float dy = i_y -keypoint.pt.y;
            if(dx*dx+dy*dy<radius*radius)
            {
                if(img.at<float>(i_y,i_x)>0.1)
                {
                    cv::Point p(i_x,i_y);
                    points.push_back(p);
                }
            }
        }
    }
    float max_value = M_PI/2.+getOrientation(points);
    ROS_INFO("orientations %f",max_value);
    return max_value;

}
float RailsDetection::computeBlobOrientation(const cv::Mat& img, cv::KeyPoint keypoint, float radius)
{
    ROS_INFO("WRONG computeBlobOrientation");
    std::map<float,int> orientations;
    for(int i_x = keypoint.pt.x -radius; i_x <keypoint.pt.x + radius; i_x++)
    {
        for(int i_y = keypoint.pt.y -radius; i_y <keypoint.pt.y + radius; i_y++)
        {
            float dx = i_x -keypoint.pt.x;
            float dy = i_y -keypoint.pt.y;
            if(dx*dx+dy*dy<radius*radius)
            {
                float orientation = img.at<float>(i_y,i_x);
                orientations[orientation]++;
            }
        }
    }
    int max_count = -1;
    float max_value = -1.;
    for(auto& o : orientations)
    {
        //std::cout << kv.first << " has value " << kv.second << std::endl;
        if(o.first != -1 && o.second > max_count)
        {
            max_count = o.second;
            max_value = o.first;
        }
    }

    //std::cout <<" max blobl orient value " << max_value << std::endl;
    return max_value;
}

void RailsDetection::computeBlobOrientations(const cv::Mat& max_orientations, const std::vector<cv::KeyPoint>& keypoints, std::vector<float>& blob_orientations)
{
    for(cv::KeyPoint keypoint : keypoints)
    {
        blob_orientations.push_back(computeBlobOrientationPCA(max_orientations, keypoint, keypoint.size));
    }
}


void RailsDetection::fitLineToBlob(const cv::Mat& max_orientations, const std::vector<cv::KeyPoint>& keypoints, const std::vector<float>& blob_orientations, std::vector<std::pair<cv::Point2i,cv::Point2i>>& lines)
{
    cv::Mat img_filtered_orientations;
    max_orientations.copyTo(img_filtered_orientations);
    img_filtered_orientations.setTo(0);

    cv::Mat img_lines;
    max_orientations.copyTo(img_lines);
    img_lines.setTo(0);

    for(int i_keypoint = 0; i_keypoint<keypoints.size(); i_keypoint++)
    {
        cv::KeyPoint keypoint = keypoints[i_keypoint];
        std::vector<cv::Point2f> line_points;
        float orientation = blob_orientations[i_keypoint];
        int radius = 20;
        for(int i_x = keypoint.pt.x -radius; i_x <keypoint.pt.x + radius; i_x++)
        {
            for(int i_y = keypoint.pt.y -radius; i_y <keypoint.pt.y + radius; i_y++)
            {
                float dx = i_x -keypoint.pt.x;
                float dy = i_y -keypoint.pt.y;
                if(dx*dx+dy*dy<radius*radius)
                {
                    if((std::fmod(std::abs(orientation - max_orientations.at<float>(i_y,i_x)),M_PI) < 0.2*M_PI) && (max_orientations.at<float>(i_y,i_x)>= 0.))
                    {
                        img_filtered_orientations.at<float>(i_y,i_x) = 1.0;
                        line_points.push_back(cv::Point2f(i_x,i_y));
                    }
                }
            }
        }


        cv::Vec4f fitted_line;
        cv::fitLine(line_points,fitted_line,CV_DIST_WELSCH,0,0.01,0.01);


        float scale = 1.5*keypoint.size;
        cv::Point2i p1(fitted_line[2]+scale*fitted_line[0],fitted_line[3]+scale*fitted_line[1]);
        cv::Point2i p2(fitted_line[2]-scale*fitted_line[0],fitted_line[3]-scale*fitted_line[1]);

        lines.push_back(std::pair<cv::Point2i,cv::Point2i>(p1,p2));
        cv::line(img_lines,p1,p2,cv::Scalar(255));
        //        cv::line(img,Point(gray.cols-1,righty),Point(0,lefty),Scalar(255,0,0),2);

    }

    cv::namedWindow("detected img_lines",cv::WINDOW_NORMAL);
    cv::imshow("detected img_lines", img_lines);
    //cv::namedWindow("img_filtered_orientations", cv::WINDOW_NORMAL);
    //cv::imshow("img_filtered_orientations", img_filtered_orientations );

}

#define DEBUG

int RailsDetection::detectRails(cv::Mat& cv_img)
{

    cv::medianBlur(cv_img,cv_img,3);
#ifdef DEBUG
    cv::namedWindow("grid input image",cv::WINDOW_NORMAL);
    cv::imshow("grid input image", cv_img);
#endif

    cv::Mat grid_diff;
    thresholdedDistance(cv_img,grid_diff);

    cv::threshold(grid_diff,grid_diff,0.6,0,cv::THRESH_TRUNC);
    cv::normalize(grid_diff,grid_diff,0.0,1.0,cv::NORM_MINMAX);
#ifdef DEBUG
    cv::namedWindow("grid_diff",cv::WINDOW_NORMAL);
    cv::imshow("grid_diff", grid_diff);
#endif

    cv::Mat feature_diff;
    cv::Mat max_orientations;

    computeRailSupport(grid_diff,feature_diff,max_orientations);
    cv::threshold(grid_diff,grid_diff,0.1,1.0,cv::THRESH_BINARY);
#ifdef DEBUG
    cv::namedWindow("feature_diff",cv::WINDOW_NORMAL);
    cv::imshow("feature_diff", feature_diff);
#endif

    cv::medianBlur(feature_diff,feature_diff,3);
    cv::normalize(feature_diff,feature_diff,0.0,1.0,cv::NORM_MINMAX);

#ifdef DEBUG
    cv::namedWindow("feature_diff_blurred",cv::WINDOW_NORMAL);
    cv::imshow("feature_diff_blurred", feature_diff);
#endif

    cv::normalize(feature_diff,feature_diff,0.0,1.0,cv::NORM_MINMAX);
    int erosion_size = 2;
    cv::Mat element_dilate = cv::getStructuringElement( 1,
                                                        cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                        cv::Point( erosion_size, erosion_size ) );
    cv::dilate( feature_diff, feature_diff, element_dilate );

    cv::threshold(feature_diff,feature_diff,0.0,1.0,cv::THRESH_BINARY_INV);

    cv::normalize(feature_diff,feature_diff,0.0,255.0,cv::NORM_MINMAX);
    cv::Mat converted_feature_diff;
    feature_diff.convertTo(converted_feature_diff,CV_8UC1);

    std::vector<cv::KeyPoint> keypoints;
    detectBlobs(feature_diff,keypoints);
    cv::Mat im_with_keypoints;
    std::cout<<"n_blobs:"<<keypoints.size()<<std::endl;
    cv::drawKeypoints( converted_feature_diff, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

#ifdef DEBUG
    cv::namedWindow("keypoints", cv::WINDOW_NORMAL);
    cv::imshow("keypoints", im_with_keypoints );
#endif

    cv::normalize(cv_img,cv_img,0.0,255.0,cv::NORM_MINMAX);
    cv::Mat converted_grid;
    cv_img.convertTo(converted_grid,CV_8UC1);
    cv::drawKeypoints( converted_grid, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

#ifdef DEBUG
    cv::namedWindow("keypoints2", cv::WINDOW_NORMAL);
    cv::imshow("keypoints2", im_with_keypoints );
#endif

    for(cv::KeyPoint keypoint : keypoints)
    {
        cv::circle(converted_grid, keypoint.pt, keypoint.size ,cv::Scalar(255));
    }


    std::vector<float> blob_orientations;
    computeBlobOrientations(max_orientations, keypoints, blob_orientations);

    cv::Mat feature_rec;
    std::vector<std::pair<cv::Point2i,cv::Point2i>> detected_rails;
    detectRectangle(grid_diff,feature_rec,keypoints, blob_orientations, detected_rails);


    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "/map";

    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.01;
    line_list.scale.y = 0.01;
    line_list.scale.z = 0.01;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    cv::Mat final_image_grey;
    cv_img.copyTo(final_image_grey);

#ifdef DEBUG
    cv::namedWindow("final_image_grey", cv::WINDOW_NORMAL);
    cv::imshow("final_image_grey", final_image_grey );
#endif

    cv::Mat final_image;
    cv::normalize(final_image_grey,final_image_grey,0.0,255,cv::NORM_MINMAX);
    final_image_grey.convertTo(final_image, CV_8UC1);
    cv::Mat detected_lines;
    cv_img.copyTo(detected_lines);
    cv::normalize(detected_lines,detected_lines,0.0,0.3,cv::NORM_MINMAX);

    geometry_msgs::Point p;
    p.z = 0.1;
    float res = grid_map_.getResolution();
    float pos_x = grid_map_.getPosition()[0];
    float pos_y = grid_map_.getPosition()[1];

    int i_line = 0;
    for(std::pair<cv::Point2i,cv::Point2i> line : detected_rails)
    {
        cv::line(final_image, line.first, line.second, cv::Scalar(255));
        cv::line(detected_lines, line.first, line.second, cv::Scalar(1), 1, CV_AA);
        geometry_msgs::Point p1;
        p1.x = pos_x-res*(float)(line.first.y-final_image.size[0]*0.5);
        p1.y = pos_y-res*(float)(line.first.x-final_image.size[1]*0.5);
        p = p1;
        p.z = 0.1;
        line_list.points.push_back(p);
        geometry_msgs::Point p2;
        p2.x = pos_x-res*(float)(line.second.y-final_image.size[0]*0.5);
        p2.y = pos_y-res*(float)(line.second.x-final_image.size[1]*0.5);
        p = p2;
        p.z = 0.1;
        line_list.points.push_back(p);


        //publish center to worldmodel
        hector_worldmodel_msgs::PosePercept pp;

        float angle = keypoints[i_line].angle;//blob_orientations[i_line];
        pp.header.frame_id= "/map";
        pp.info.class_id= "rail";
        pp.info.class_support=1;
        pp.info.object_support=1;
        pp.pose.pose.position.x= 0.5*(p1.x+p2.x);
        pp.pose.pose.position.y= 0.5*(p1.y+p2.y);
        pp.pose.pose.position.z= 0.1;
        pp.pose.pose.orientation.x= sin(angle/2.);
        pp.pose.pose.orientation.y = sin(angle/2.);
        pp.pose.pose.orientation.z= 0;
        pp.pose.pose.orientation.w= cos(angle/2.);

        pose_percept_publisher_.publish(pp);
        ROS_INFO("Rail postions published");
        i_line++;
    }

    marker_publisher_.publish(line_list);
    cv::normalize(feature_rec,feature_rec,0.0,1.0,cv::NORM_MINMAX);

#ifdef DEBUG
    cv::namedWindow("feature_rec",cv::WINDOW_NORMAL);
    cv::imshow("feature_rec", feature_rec);
    cv::namedWindow("detected_lines",cv::WINDOW_NORMAL);
    cv::imshow("detected_lines", detected_lines);
    cv::waitKey(0);
#endif

    return 0;
}

void rotatePoint(cv::Point2f& p, cv::Point2f c, float a)
{
    float x = (p.x-c.x)*cos(a) -(p.y-c.y)*sin(a) + c.x;
    float y = (p.x-c.x)*sin(a) +(p.y-c.y)*cos(a) + c.y;
    p.x = x;
    p.y = y;
}
void RailsDetection::detectRectangle(const cv::Mat& img_in, cv::Mat& img_support, std::vector<cv::KeyPoint>& keypoints, const std::vector<float>& blob_orientations, std::vector<std::pair<cv::Point2i,cv::Point2i>>& rails)
{
    img_in.copyTo(img_support);
    img_support.setTo(0);
    float w = track_width_/0.05;
    float l = track_length_/0.05;
    int i_key = 0;
    cv::Point2f p_zero(0,0);
    for(cv::KeyPoint& key : keypoints)
    {
        float orientation = blob_orientations[i_key];
        int radius = key.size;
        float orientation_delta = 0.1;
        float orientation_steps = 10;
        float max_orientation = 0.;
        cv::Point2i max_point;
        cv::Point2f p1_max;
        cv::Point2f p2_max;
        cv::Point2f p3_max;
        cv::Point2f p4_max;
        float max_support = 0;

        for(int i_orient = -orientation_steps/2; i_orient<orientation_steps/2; i_orient++)
        {
            orientation = blob_orientations[i_key] +orientation_delta*(float)i_orient;
            cv::Point2f p1_l(-0.5*w,-0.5*l);
            cv::Point2f p2_l(-0.5*w, 0.5*l);
            cv::Point2f p3_l( 0.5*w,-0.5*l);
            cv::Point2f p4_l( 0.5*w, 0.5*l);
            rotatePoint(p1_l,p_zero, orientation);
            rotatePoint(p2_l,p_zero, orientation);
            rotatePoint(p3_l,p_zero, orientation);
            rotatePoint(p4_l,p_zero, orientation);
            ROS_DEBUG("angle %f",key.angle);
            unsigned int i_r_min = key.pt.y-radius > 0 ?  key.pt.y-radius: 0 ;
            unsigned int i_r_max = key.pt.y+radius < img_in.cols ?  key.pt.y+radius: img_in.cols ;
            unsigned int i_c_min = key.pt.x-radius > 0 ?  key.pt.x-radius: 0 ;
            unsigned int i_c_max = key.pt.x+radius < img_in.cols ?  key.pt.x+radius: img_in.cols;

            for(unsigned int i_r = i_r_min; i_r < i_r_max; ++i_r)
            {
                for(unsigned int i_c = i_c_min; i_c < i_c_max; ++i_c)
                {
                    float support = 0;


                    cv::Point2f p1(i_c,i_r);
                    cv::Point2f p2(i_c,i_r);
                    cv::Point2f p3(i_c,i_r);
                    cv::Point2f p4(i_c,i_r);
                    p1 += p1_l;
                    p2 += p2_l;
                    p3 += p3_l;
                    p4 += p4_l;
                    cv::LineIterator line_it(img_in, p1, p2);
                    for(int i_it = 0; i_it<line_it.count; i_it++)
                    {
                        line_it++;
                        float res = img_in.at<float>(line_it.pos());
                        if(res > 0.1)
                            res = 0.1;
                        support += res;
                    }
                    line_it =cv::LineIterator(img_in, p3, p4);
                    for(int i_it = 0; i_it<line_it.count; i_it++)
                    {
                        line_it++;
                        float res = img_in.at<float>(line_it.pos());
                        if(res > 0.1)
                            res = 0.1;
                        support += res;
                    }
                    if(support > max_support)
                    {
                        max_support = support;
                        max_point.y = i_r;
                        max_point.x = i_c;
                        p1_max = p1;
                        p2_max = p2;
                        p3_max = p3;
                        p4_max = p4;
                        max_orientation = orientation;
                        key.angle = orientation;
                    }
                    float s3 = support*support*support;
                    if(s3 > img_support.at<float>(i_r,i_c))
                        img_support.at<float>(i_r,i_c) = s3;
                }
            }
        }
        std::pair<cv::Point2i,cv::Point2i> p12;
        p12.first = p1_max;
        p12.second = p2_max;
        std::pair<cv::Point2i,cv::Point2i> p34;
        p34.first = p3_max;
        p34.second = p4_max;
        rails.push_back(p12);
        rails.push_back(p34);

    }
}

void RailsDetection::dynamic_recf_cb(hector_rails_detection::HectorRailsDetectionConfig &config, uint32_t level)
{
    track_length_ = config.track_length;
    track_width_ = config.track_width;
    rail_width_ = config.rail_width;
    n_rails_ = config.n_rails;
    gradient_z_min_= config.gradient_z_min;
    gradient_z_max_= config.gradient_z_max;
    gradient_dif_min_= config.gradient_dif_min;
    gradient_dif_max_= config.gradient_dif_max;
    rail_support_feature_thresh_ = config.rail_support_feature_thresh;
    rail_support_feature_section_thresh_ = config.rail_support_feature_section_thresh;
}

}

/*
int main(int argc, char **argv)
{

    cv::Mat cv_img;
    //cv_img = cv::imread("/home/kevin/Downloads/Gray_Image.png");
    cv_img = cv::imread("/home/kevin/Downloads/Gray_Image_rotated.png");
    cv::Mat conv_img;
    cv::cvtColor(cv_img, conv_img, CV_RGB2GRAY);
    conv_img.convertTo(conv_img,CV_32FC1);
    std::cout<<type2str(conv_img.type())<<std::endl;

    //detectRails();
    detectRails(conv_img);
}
*/
