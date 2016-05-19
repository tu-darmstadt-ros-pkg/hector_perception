//=================================================================================================
// Copyright (c) 2014, Stefan Kohlbrecher, TU Darmstadt
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

#ifndef OCTOMAP_CV_INTERFACE_H_
#define OCTOMAP_CV_INTERFACE_H_

/* TODO
 * map_width: originally set from nav_msgs
 *
 */



//#include <sensor_msgs/PointCloud2.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo/photo.hpp>

#include <cv_image_proc/cv_image_convert.h>

namespace octomap_cv_interface
{


class OctomapCvInterface
{
public:

  OctomapCvInterface()
    : m_octree(0)
  {
    m_octree = new octomap::OcTree(0.05);
  }

  bool fromFile(const std::string& filename){
    if (filename.length() <= 3)
      return false;

    std::string suffix = filename.substr(filename.length()-3, 3);
    if (suffix== ".bt"){
      if (!m_octree->readBinary(filename)){
        return false;
      }
    } else if (suffix == ".ot"){
      octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read(filename);
      if (!tree){
        return false;
      }
      if (m_octree){
        delete m_octree;
        m_octree = NULL;
      }
      m_octree = dynamic_cast<octomap::OcTree*>(tree);
      if (!m_octree){
        ROS_ERROR("Could not read OcTree in file, currently there are no other types supported in .ot");
        return false;
      }

    } else{
      return false;
    }

    m_treeDepth = m_octree->getTreeDepth();
    m_maxTreeDepth = m_treeDepth;

    m_multires2DScale = 1 << (m_treeDepth - m_maxTreeDepth);
  }

  void printInfo()
  {
    std::cout << "mem_usage: " << m_octree->memoryUsage() << " size: " << m_octree->size() << " depth: " << m_octree->getTreeDepth() << "\n";
  }

  void process()
  {
    //cv::Mat heightmap;
    //this->retrieveHeightMap(heightmap);
  }

  inline unsigned mapIdx(int i, int j) const {
    return map_width * j + i;
  }

  inline unsigned mapIdx(const octomap::OcTreeKey& key) const {
    return mapIdx((key[0] - m_paddedMinKey[0]) / m_multires2DScale,
                  (key[1] - m_paddedMinKey[1]) / m_multires2DScale);

  }

  void update2DMap(const octomap::OcTree::iterator& it, bool occupied){

    // update 2D map (occupied always overrides):

    if (it.getDepth() == m_maxTreeDepth){
      unsigned idx = mapIdx(it.getKey());
      if (occupied){
        //m_gridmap.data[mapIdx(it.getKey())] = 100;
      }//else if (m_gridmap.data[idx] == -1){
        //m_gridmap.data[idx] = 0;
      //}

    } else{
      int intSize = 1 << (m_maxTreeDepth - it.getDepth());
      octomap::OcTreeKey minKey=it.getIndexKey();
      for(int dx=0; dx < intSize; dx++){
        int i = (minKey[0]+dx - m_paddedMinKey[0])/m_multires2DScale;
        for(int dy=0; dy < intSize; dy++){
          unsigned idx = mapIdx(i, (minKey[1]+dy - m_paddedMinKey[1])/m_multires2DScale);
          if (occupied){
            //m_gridmap.data[idx] = 100;
          }//else if (m_gridmap.data[idx] == -1){
            //m_gridmap.data[idx] = 0;
          //}
        }
      }
    }
  }




  bool retrieveHeightMap(cv::Mat& heightmap, cv::Mat& free_map){



    ros::WallTime startTime = ros::WallTime::now();
    size_t octomapSize = m_octree->size();
    // TODO: estimate num occ. voxels for size of arrays (reserve)
    if (octomapSize <= 1){
      ROS_WARN("Nothing to publish, octree is empty");
      return false;
    }

    //bool publishFreeMarkerArray = m_publishFreeSpace && (m_latchedTopics || m_fmarkerPub.getNumSubscribers() > 0);
    //bool publishMarkerArray = (m_latchedTopics || m_markerPub.getNumSubscribers() > 0);
    //bool publishPointCloud = (m_latchedTopics || m_pointCloudPub.getNumSubscribers() > 0);
    //bool publishBinaryMap = (m_latchedTopics || m_binaryMapPub.getNumSubscribers() > 0);
    //bool publishFullMap = (m_latchedTopics || m_fullMapPub.getNumSubscribers() > 0);
    //m_publish2DMap = (m_latchedTopics || m_mapPub.getNumSubscribers() > 0);

    // init markers for free space:
    //visualization_msgs::MarkerArray freeNodesVis;
    // each array stores all cubes of a different size, one for each depth level:
    //freeNodesVis.markers.resize(m_treeDepth+1);

    geometry_msgs::Pose pose;
    pose.orientation = tf::createQuaternionMsgFromYaw(0.0);


    // call pre-traversal hook:
    //handlePreNodeTraversal(rostime);

    double minX, minY, minZ, maxX, maxY, maxZ;
    m_octree->getMetricMin(minX, minY, minZ);
    m_octree->getMetricMax(maxX, maxY, maxZ);

    std::cout << "min: " << minX << "\n";
    std::cout << "max: " << maxX << "\n";


    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey = m_octree->coordToKey(minPt, m_octree->getTreeDepth());
    octomap::OcTreeKey maxKey = m_octree->coordToKey(maxPt, m_octree->getTreeDepth());

    m_paddedMinKey = minKey;

        //int width = (paddedMaxKey[0] - m_paddedMinKey[0])/m_multires2DScale +1;
        //int height = (paddedMaxKey[1] - m_paddedMinKey[1])/m_multires2DScale +1;

    int width = (maxKey[0] - minKey[0])/m_multires2DScale +1;
    int height = (maxKey[1] - minKey[1])/m_multires2DScale +1;
    map_width = width;

    cv::Mat cv_img(height, width, CV_32FC1, cv::Scalar(-std::numeric_limits<float>::max()));
    cv::Mat free_img(height, width, CV_32FC1, cv::Scalar(std::numeric_limits<float>::max()));
    //cv::Mat free_img(height, width, CV_32FC1, cv::Scalar(1.49));
        //
    //cv::Mat cv_img(height, width, CV_32FC1, cv::Scalar(-1.0));

    //std::cout << "grid w: " << width << " h " << height << "\n";


  for (octomap::OcTree::iterator it = m_octree->begin(m_maxTreeDepth),
        end = m_octree->end(); it != end; ++it)
    {
      //bool inUpdateBBX = isInUpdateBBX(it);

      // call general hook:
      //handleNode(it);
      //if (inUpdateBBX)
      //  handleNodeInBBX(it);



      //if (m_octree->isNodeOccupied(*it)){

    bool occupied = m_octree->isNodeOccupied(*it);
        double z = it.getZ();

        if (z < 1.5){


          if (it.getDepth() == m_maxTreeDepth){
            unsigned idx = mapIdx(it.getKey());
            //int idx = mapIdx(it.getKey());
            //std::cout << idx << " z: " << z <<"\n";

            if (occupied){
              if (z > cv_img.at<float>(idx)){
                cv_img.at<float>(idx) = z;
              }
            }else{
              if (z < free_img.at<float>(idx)){
                free_img.at<float>(idx) = z;
              }
            }

            //std::cout << cv_img.at<float>(idx) <<"\n";
            //cv_img[idx] = z;
            //if (occupied){
            //m_gridmap.data[mapIdx(it.getKey())] = 100;
            //}//else if (m_gridmap.data[idx] == -1){
            //m_gridmap.data[idx] = 0;
            //}

          } else{
            int intSize = 1 << (m_maxTreeDepth - it.getDepth());
            octomap::OcTreeKey minKey=it.getIndexKey();
            for(int dx=0; dx < intSize; dx++){
              int i = (minKey[0]+dx - m_paddedMinKey[0])/m_multires2DScale;
              for(int dy=0; dy < intSize; dy++){
                unsigned idx = mapIdx(i, (minKey[1]+dy - m_paddedMinKey[1])/m_multires2DScale);

                if (occupied){
                  if (z > cv_img.at<float>(idx)){
                    cv_img.at<float>(idx) = z;
                  }
                }else{
                  if (z < free_img.at<float>(idx)){
                    free_img.at<float>(idx) = z;
                  }
                }
                //if (occupied){
                //m_gridmap.data[idx] = 100;
                //}//else if (m_gridmap.data[idx] == -1){
                //m_gridmap.data[idx] = 0;
                //}
              }
            }
          }
        }

        //if (z > cv_){

        //}
        /*
        if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
        {
          double size = it.getSize();
          double x = it.getX();
          double y = it.getY();
  #ifdef COLOR_OCTOMAP_SERVER
          int r = it->getColor().r;
          int g = it->getColor().g;
          int b = it->getColor().b;
  #endif

          // Ignore speckles in the map:
          if (m_filterSpeckles && (it.getDepth() == m_treeDepth +1) && isSpeckleNode(it.getKey())){
            ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
            continue;
          } // else: current octree node is no speckle, send it out

          handleOccupiedNode(it);
          if (inUpdateBBX)
            handleOccupiedNodeInBBX(it);


        }
      } else{ // node not occupied => mark as free in 2D map if unknown so far
        double z = it.getZ();
        if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
        {
          handleFreeNode(it);
          if (inUpdateBBX)
            handleFreeNodeInBBX(it);
        }
      }
      */
    //}
  }

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  std::cout << total_elapsed;

  heightmap = cv_img;

  free_map = free_img;

  return true;

  /*
  for (size_t i = 0; i < cv_img.total(); ++i){
    cv_img.at<float>(i) += 1.0;
  }
  */

    // call post-traversal hook:
    //handlePostNodeTraversal(rostime);



    //ROS_INFO("Map publishing in OctomapServer took %f sec", total_elapsed);

  }




protected:
  octomap::OcTree* m_octree;

  int m_maxTreeDepth;
  int m_treeDepth;
  int map_width;
  unsigned m_multires2DScale;
  octomap::OcTreeKey m_paddedMinKey;
  //int mapIdx;


};

}
#endif
