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
namespace hector_rails_detection
{
RailsDetection::RailsDetection()
{
    ros::NodeHandle nh("");
    elevation_map_subscriber_ = nh.subscribe("/elevation_mapping/elevation_map_global",10, &RailsDetection::elevationMapCallback, this);

    nh.param("gradient_z_min", gradient_z_min_, -0.5);
    nh.param("gradient_z_max", gradient_z_max_, 2.0);
    nh.param("gradient_dif_min", gradient_dif_min_, 0.0);
    nh.param("gradient_dif_max", gradient_dif_max_, 2.0);
    nh.param("rail_support_feature_thresh", rail_support_feature_thresh_, 1.3);
    nh.param("rail_support_feature_section_thresh", rail_support_feature_section_thresh_, 0.3);

    marker_publisher_ = nh.advertise<visualization_msgs::Marker>("hector_rail_detection/line_perecepts", 10);
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

    /*
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
    */
    max_height_ = 1.;
    min_height_ = 0.;

    if(!is_init)
    {
        ROS_INFO("map callback");
        grid_map::GridMapRosConverter::fromMessage(grid_map_msg, grid_map_);
        ROS_INFO("max %f min %f", max_height_, min_height_);
        is_init = true;
    }



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

float RailsDetection::computeRailSupportGeneric(const cv::Mat& img, int row, int col, float angle = 0.0)
{

    //angle = 3.14/2.0;
    std::vector<section> sections;

    sections.push_back(std::vector<float>{-6.5,-4.5,-1.0});
    sections.push_back(std::vector<float>{-4.0,-2.0,1.0});
    sections.push_back(std::vector<float>{-2.0,2.0,-1.0});
    sections.push_back(std::vector<float>{2.5,4.5,1.0});
    sections.push_back(std::vector<float>{4.5,6.5,-1.0});


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
            //if(img.at<float>(line_it.pos())==sec[2])
            float weight = std::abs((float)i_it-0.5*(float)line_it.count)/(float)line_it.count;
            //std::cout<<weight<<std::endl;
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
        if(pos/(neg+1E-5)>rail_support_feature_section_thresh_)
            support++;
    }
    return support;
}

float RailsDetection::computeRailSupportMultiple(const cv::Mat& img, int row, int col, float angle = 0.0)
{
    float support0 = computeRailSupportGeneric(img, row, col, angle);

    int dx0 = round(cos(angle+M_PI_2)*1.4);
    int dy0 = round(sin(angle+M_PI_2)*1.4);
    // std::cout<<"angle "<<angle<<" "<<dx0<<" "<<dy0<<std::endl;

    float support1 = computeRailSupportGeneric(img, row+dy0, col+dx0, angle);
    float support2 = computeRailSupportGeneric(img, row-dy0, col-dx0, angle);

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
                        bool valid_grad =  delta_temp < gradient_dif_max_ && delta_temp > gradient_dif_min_;
                        float delta = 0.0;
                        if (valid_height && valid_grad)
                        {
                            delta = delta_temp-gradient_dif_min_;

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
    cv::SimpleBlobDetector detector(params);

    params.filterByColor = false;

    // Detect blobs.
    keypoints.clear();
    detector.detect( blob_base, keypoints);
}

float RailsDetection::computeBlobOrientation(const cv::Mat& img, cv::KeyPoint keypoint, float radius)
{
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
    std::cout<<std::endl;
    std::cout<<std::endl;
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

    //std::cout <<" max value " << max_value << std::endl;
    return max_value;
}

void RailsDetection::computeBlobOrientations(const cv::Mat& max_orientations, const std::vector<cv::KeyPoint>& keypoints, std::vector<float>& blob_orientations)
{
    for(cv::KeyPoint keypoint : keypoints)
    {
        blob_orientations.push_back(computeBlobOrientation(max_orientations, keypoint, 15));
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

int RailsDetection::detectRails(cv::Mat& cv_img)
{
#ifdef DEBUG
    cv::namedWindow("grid input image",cv::WINDOW_NORMAL);
    cv::imshow("grid input image", cv_img);
#endif

    cv::Mat grid_diff;
    thresholdedDistance(cv_img,grid_diff);
    // cv::threshold(grid_diff,grid_diff,0.0,255,cv::THRESH_BINARY);

    //cv::Mat grid_diff_vis;
    cv::normalize(grid_diff,grid_diff,0.0,1.0,cv::NORM_MINMAX);


#ifdef DEBUG
    cv::namedWindow("grid_diff",cv::WINDOW_NORMAL);
    cv::imshow("grid_diff", grid_diff);
#endif

    //  cv::Mat grid_diff_smooth;
    cv::medianBlur(grid_diff,grid_diff,3);
    // cv::namedWindow("grid_diff_smooth",cv::WINDOW_NORMAL);
    // cv::imshow("grid_diff_smooth", grid_diff_smooth);


    cv::Mat feature_diff;
    cv::Mat max_orientations;

    computeRailSupport(grid_diff,feature_diff,max_orientations);
    cv::normalize(feature_diff,feature_diff,0.0,1.0,cv::NORM_MINMAX);

#ifdef DEBUG
    cv::namedWindow("feature_diff",cv::WINDOW_NORMAL);
    cv::imshow("feature_diff", feature_diff);
#endif

    // cv::namedWindow("max_orientations",cv::WINDOW_NORMAL);
    //   cv::imshow("max_orientations", max_orientations);


    cv::normalize(feature_diff,feature_diff,0.0,1.0,cv::NORM_MINMAX);
    int erosion_size = 2;
    cv::Mat element_dilate = cv::getStructuringElement( 1,
                                                        cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                        cv::Point( erosion_size, erosion_size ) );
    cv::dilate( feature_diff, feature_diff, element_dilate );
    //   cv::namedWindow("grid_diff_eroded",cv::WINDOW_NORMAL);
    //   cv::imshow("grid_diff_eroded", feature_diff);

    cv::threshold(feature_diff,feature_diff,0.0,1.0,cv::THRESH_BINARY_INV); //todo thresh param in dynmaic reconfigure
    // cv::namedWindow("THRESH_BINARY_INV",cv::WINDOW_NORMAL);
    // cv::imshow("THRESH_BINARY_INV", feature_diff);

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
    std::vector<std::pair<cv::Point2i,cv::Point2i>> lines;
    fitLineToBlob(max_orientations, keypoints, blob_orientations, lines);

#ifdef DEBUG
    cv::namedWindow("converted_grid", cv::WINDOW_NORMAL);
    cv::imshow("converted_grid", converted_grid );
#endif

    cv::Mat final_image_grey;
    cv_img.copyTo(final_image_grey);

#ifdef DEBUG
    cv::namedWindow("final_image_grey", cv::WINDOW_NORMAL);
    cv::imshow("final_image_grey", final_image_grey );
#endif

    cv::Mat final_image;
    cv::normalize(final_image_grey,final_image_grey,0.0,255,cv::NORM_MINMAX);
    final_image_grey.convertTo(final_image, CV_8UC1);


    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "/map";
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1;
    line_list.scale.y = 0.1;
    line_list.scale.z = 0.1;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    geometry_msgs::Point p;
    /*p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    line_list.points.push_back(p);
    p.z += 1.0;
    line_list.points.push_back(p);
*/

    ROS_INFO("length %f %f pose %f %f",grid_map_.getLength()[0],grid_map_.getLength()[1],grid_map_.getPosition()[0],grid_map_.getPosition()[1]);
    ROS_INFO("res %f ",grid_map_.getResolution());

    float res = grid_map_.getResolution();
    float pos_x = grid_map_.getPosition()[0];
    float pos_y = grid_map_.getPosition()[1];
    for(std::pair<cv::Point2i,cv::Point2i> line : lines)
    {
        //  cv::Point2i p1(line[2]+12*line[0],line[3]+12*line[1]);
        //  cv::Point2i p2(line[2]-12*line[0],line[3]-12*line[1]);
        cv::line(final_image, line.first, line.second, cv::Scalar(255));
        p.x = pos_x-res*(float)(line.first.y-final_image.size[0]*0.5);
        p.y = pos_y-res*(float)(line.first.x-final_image.size[1]*0.5);
        line_list.points.push_back(p);
        p.x = pos_x-res*(float)(line.second.y-final_image.size[0]*0.5);
        p.y = pos_y-res*(float)(line.second.x-final_image.size[1]*0.5);
        line_list.points.push_back(p);

    }

    marker_publisher_.publish(line_list);

//#ifdef DEBUG
    cv::namedWindow("final_image", cv::WINDOW_NORMAL);
    cv::imshow("final_image", final_image );
//#endif
    /*
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(converted_grid_diff, lines, 2, M_PI/180, 70, 0, 0 ); // draw lines
    //cv::HoughLines(converted_grid_diff, lines, 2, M_PI/180, 30, 0, 0 ); // draw lines
    cv::Mat cdst(cv_img.rows,cv_img.cols, CV_32FC1, cv::Scalar(0));


    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = (x0 + 1000*(-b));
        pt1.y = (y0 + 1000*(a));
        pt2.x = (x0 - 1000*(-b));
        pt2.y = (y0 - 1000*(a));
        cv::line( cdst, pt1, pt2, cv::Scalar(1), 1, CV_AA);

        std::cout<<"line:"<<pt1<<" "<<pt2<<std::endl;
    }

    // cv::imshow("source", src);
    cv::namedWindow("detected lines",cv::WINDOW_NORMAL);
    cv::imshow("detected lines", cdst);
*/
    /*
    std::vector<cv::Vec4i> linesp;
    cv::HoughLinesP(converted_grid, linesp, 1, M_PI/180, 1, 10, 0 ); // draw lines
    //cv::HoughLinesP(converted_grid_diff, linesp, 1, M_PI/180, 2, 15, 0 ); // draw lines
    cv::Mat cdstp(cv_img.rows,cv_img.cols, CV_32FC1, cv::Scalar(0));


    std::cout<<"detected "<<linesp.size()+1<<" lines"<<std::endl;
    for (size_t i = 0; i < linesp.size(); ++i)
    {
        cv::Vec4i l = linesp[i];
        cv::line(cdstp, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(1), 1, CV_AA);
    }

    cv::namedWindow("detected linesp",cv::WINDOW_NORMAL);
    cv::imshow("detected linesp", cdstp);

*/


    cv::waitKey(0);
    return 0;
}

void RailsDetection::dynamic_recf_cb(hector_rails_detection::HectorRailsDetectionConfig &config, uint32_t level)
{
    ROS_INFO("dynamic reconf");
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
