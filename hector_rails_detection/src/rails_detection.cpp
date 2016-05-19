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
#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo/photo.hpp>

typedef std::vector<float> section;

std::string type2str(int type) {
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



float computeRailSupportGeneric(const cv::Mat& img, int row, int col, float angle = 0.0)
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
            if(sec[2]*(img.at<float>(line_it.pos())-127.5) > 0.0)
            {
                pos += weight;
            }
            else
            {
                neg += weight;
            }
            line_it++;
        }
        if(pos/(neg+1E-5)>0.5)
            support++;
    }
    return support;
}


float computeRailSupportMultiple(const cv::Mat& img, int row, int col, float angle = 0.0)
{
    float support0 = computeRailSupportGeneric(img, row, col, angle);

    int dx0 = round(cos(angle+M_PI_2)*1.4);
    int dy0 = round(sin(angle+M_PI_2)*1.4);
    // std::cout<<"angle "<<angle<<" "<<dx0<<" "<<dy0<<std::endl;

    float support1 = computeRailSupportGeneric(img, row+dy0, col+dx0, angle);
    float support2 = computeRailSupportGeneric(img, row-dy0, col-dx0, angle);

    return (2.0*support0+support1+support2)/4.0;


}

float computeRailSupport(const cv::Mat& img, int row, int col, float angle = 0.0)
{

    //angle = 3.14/2.0;
    int radius = 6;
    float dx = (cos(angle)*(float)radius);
    float dy = (sin(angle)*(float)radius);
    float scale_factor = (float)radius/std::max(abs(dx),abs(dy));
    dx *= scale_factor;
    dy *= scale_factor;

    cv::LineIterator it(img, cv::Point2f(-dx+col,-dy+row),cv::Point2f(dx+col,dy+row));



    std::vector<std::vector<float>> bins = {{-1,-1},{1,1},{-1,-1,-1,-1,-1},{1,1},{-1,-1}};
    int n_size = 13;
    float support = 0.0;
    int x_offset = -(13-1)/2;
    for(int i_bin = 0; i_bin<bins.size(); i_bin++)
    {
        float inner_support = 0;
        for(int i_inner = 0; i_inner<bins[i_bin].size(); i_inner++)
        {
            //if(bins[i_bin][i_inner]*(img.at<float>(row,col+x_offset)-127.5) > 0.0)
            if(bins[i_bin][i_inner]*(img.at<float>(it.pos())-127.5) > 0.0)
            {
                inner_support++;
            }
            x_offset++;
            it++;
        }
        if(inner_support>(bins[i_bin].size()-1)/2)
            support++;
    }

    return support;
}


int detectRails(cv::Mat& cv_img)
{
    cv::namedWindow("grid",cv::WINDOW_NORMAL);
    cv::imshow("grid", cv_img);


    //std::cout<<cv_img<<std::endl;
    cv::Mat grid_diff;
    cv_img.copyTo(grid_diff);
    grid_diff.setTo(0);

    for(unsigned int i_r = 5; i_r < cv_img.rows-5; ++i_r)
    {
        for(unsigned int i_c = 5; i_c < cv_img.cols-5; ++i_c)
        {
            for(int i_x = -2; i_x < 3; ++i_x)
            {
                for(int i_y = -2; i_y < 3; ++i_y)
                {
                    if(!(i_x ==0 && i_y == 0))
                    {
                        float delta = (cv_img.at<float>(i_r,i_c)-cv_img.at<float>(i_r+i_x,i_c+i_y));

                        if (delta <20.0 && delta >0.0)
                            delta = 0.0;
                        // if (delta >50.0)
                        //     delta = 50.0;
                        grid_diff.at<float>(i_r,i_c) += delta;
                    }
                }
            }
        }
    }


    for(unsigned int i_r = 5; i_r < cv_img.rows-5; ++i_r)
    {
        for(unsigned int i_c = 5; i_c < cv_img.cols-5; ++i_c)
        {
            if (grid_diff.at<float>(i_r,i_c) > 0.0)
                grid_diff.at<float>(i_r,i_c) = 255.0;
        }
    }

    cv::namedWindow("grid_diff",cv::WINDOW_NORMAL);
    cv::imshow("grid_diff", grid_diff);


    cv::Mat feature_diff;
    cv_img.copyTo(feature_diff);
    feature_diff.setTo(0);
    for(unsigned int i_r = 7; i_r < cv_img.rows-7; ++i_r)
    {
        for(unsigned int i_c = 7; i_c < cv_img.cols-7; ++i_c)
        {
            std::vector<float> supports;
            for(float i_orient = 0.; i_orient<8.; i_orient++)
                supports.push_back(computeRailSupportMultiple(grid_diff,i_r,i_c,M_PI*i_orient/8.0));
            float support = *std::max_element(supports.begin(),supports.end());

            if(support > 4.3)
                feature_diff.at<float>(i_r,i_c) =(1.0/(pow(5.0,2)))*support*support;

        }
    }

    cv::namedWindow("feature_diff",cv::WINDOW_NORMAL);
    cv::imshow("feature_diff", feature_diff);


    for(unsigned int i_r = 5; i_r < feature_diff.rows-5; ++i_r)
    {
        for(unsigned int i_c = 5; i_c < feature_diff.cols-5; ++i_c)
        {
            if (feature_diff.at<float>(i_r,i_c) > 0.0)
                feature_diff.at<float>(i_r,i_c) = 255.0;
        }
    }








    int erosion_size = 2;
    cv::Mat element = cv::getStructuringElement( 1,
                                                 cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                 cv::Point( erosion_size, erosion_size ) );

    /// Apply the erosion operation
    cv::dilate( feature_diff, feature_diff, element );
    //cv::dilate( feature_diff, feature_diff, element );
    //cv::dilate( feature_diff, feature_diff, element );
    cv::namedWindow("grid_diff_eroded",cv::WINDOW_NORMAL);
    cv::imshow("grid_diff_eroded", feature_diff);


    for(unsigned int i_r = 0; i_r < feature_diff.rows; ++i_r)
    {
        for(unsigned int i_c = 0; i_c < feature_diff.cols; ++i_c)
        {

                feature_diff.at<float>(i_r,i_c) = 255.0- feature_diff.at<float>(i_r,i_c);
        }
    }
    cv::Mat converted_grid_diff;
    feature_diff.convertTo(converted_grid_diff,CV_8UC1);





    cv::Mat blob_base;
    feature_diff.convertTo(blob_base,CV_8UC1);
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
    std::vector<cv::KeyPoint> keypoints;
    detector.detect( blob_base, keypoints);

    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    cv::Mat im_with_keypoints;
    // cv_img.copyTo(im_with_keypoints);
    //  im_with_keypoints.setTo(0);
    std::cout<<"n_blobs:"<<keypoints.size()<<std::endl;
    cv::drawKeypoints( blob_base, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    // Show blobs
    cv::namedWindow("keypoints", cv::WINDOW_NORMAL);
    cv::imshow("keypoints", im_with_keypoints );

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
    cv::HoughLinesP(converted_grid_diff, linesp, 1, M_PI/180, 1, 10, 0 ); // draw lines
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


int detectRails()
{

    /*octomap_cv_interface::OctomapCvInterface test;

    //test.fromFile("/home/kohlbrecher/logs/argos/obstacles/elevated_1.ot");
    //test.fromFile("/home/kohlbrecher/logs/argos/obstacles/taurob_near_stairs.ot");
    test.fromFile("/home/kevin/Downloads/octo_1.ot");
    test.printInfo();

    cv::Mat cv_img;
    cv::Mat free_map;
    test.retrieveHeightMap(cv_img, free_map);
    double max = cv_img.at<float>(0,0);
    for(int i = 0; i < cv_img.rows; i++)
    {
        for(int j = 0; j < cv_img.cols; j++)
        {
            float val = cv_img.at<float>(i,j);
            if(val > max)
                max = val;

        }
    }

    for(int i = 0; i < cv_img.rows; i++)
    {
        for(int j = 0; j < cv_img.cols; j++)
        {
            float val = cv_img.at<float>(i,j);
            cv_img.at<float>(i,j) = val*255.0/max;
            if(cv_img.at<float>(i,j)<0) cv_img.at<float>(i,j) = 0;
        }
    }

    cv::imwrite( "/home/kevin/Downloads/Gray_Image.png", cv_img);

    detectRails(cv_img);
*/
    return 0;
}




int main(int argc, char **argv)
{

    cv::Mat cv_img;
    //cv_img = cv::imread("/home/kevin/Downloads/Gray_Image.png");
    cv_img = cv::imread("/home/kevin/Downloads/Gray_Image_rotated.png");
    cv::Mat conv_img;//(cv_img.rows,cv_imgs.cols,CV_32FC1);

    cv::cvtColor(cv_img, conv_img, CV_RGB2GRAY);
    conv_img.convertTo(conv_img,CV_32FC1);
    std::cout<<type2str(conv_img.type())<<std::endl;

    //detectRails();
    detectRails(conv_img);
}
