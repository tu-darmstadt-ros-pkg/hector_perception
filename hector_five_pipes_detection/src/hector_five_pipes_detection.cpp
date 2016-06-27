#include <ros/ros.h>
#include <hector_five_pipes_detection/hector_five_pipes_detection.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace hector_five_pipes_detection{
HectorFivePipesDetection::HectorFivePipesDetection(){
    ROS_DEBUG ("HectorFivePipesDetection started");
    ros::NodeHandle nh("");

    //load params
    nh.param("passThroughXMin", passThroughXMin_, 0.1);
    nh.param("passThroughYMin", passThroughYMin_, 0.1);
    nh.param("passThroughZMin", passThroughZMin_, 0.1);
    nh.param("passThroughXMax", passThroughXMax_, 0.1);
    nh.param("passThroughYMax", passThroughYMax_, 0.0);
    nh.param("passThroughZMax", passThroughZMax_, 2.0);
    nh.param("voxelGridX", voxelGridX_, 0.05);
    nh.param("voxelGridY", voxelGridY_, 0.05);
    nh.param("voxelGridZ", voxelGridZ_, 0.05);
    nh.param("planeSegDistTresh", planeSegDistTresh_, 0.03);
    nh.param("numberPointsThresh", numberPointsThresh_, 1000);
    nh.param("clusterTolerance", clusterTolerance_, 0.03);
    nh.param("minClusterSize", minClusterSize_, 10);
    nh.param("maxClusterSize", maxClusterSize_, 1000);
    nh.param("searchRadius", searchRadius_, 0.2);

    nh.param("worldFrame", worldFrame_, std::string("/world"));

    dynamic_recf_type = boost::bind(&HectorFivePipesDetection::dynamic_recf_cb, this, _1, _2);
    dynamic_recf_server.setCallback(dynamic_recf_type);

    orginal_pub_debug_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_five_pipe_detection/input_cloud_debug", 100, true);
    after_pass_through_pub_debug_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_five_pipe_detection/after_pass_through_debug", 100, true);
    after_voxel_grid_pub_debug_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_five_pipe_detection/after_voxel_gird_debug", 100, true);
    final_cloud_pub_debug_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_five_pipe_detection/final_cloud_pub_debug", 100, true);
    plane_pub_debug_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_five_pipe_detection/plane_pub_debug", 100, true);
    cloud_filtered_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_five_pipe_detection/cylinder_cloud_debug", 100, true);
    cluster_pub_debug_= nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("/hector_five_pipe_detection/cluster_cloud_debug", 100, true);
    five_pipes_pos_pub_= nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("/hector_five_pipe_detection/five_pipes_positions", 100, true);
    cluster_centers_pub_= nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("/hector_five_pipe_detection/cloud_centers", 100, true);
    posePercept_pub_= nh.advertise<hector_worldmodel_msgs::PosePercept>("/worldmodel/pose_percept", 0);

    pointcloud_sub_ = nh.subscribe("/worldmodel_main/pointcloud_vis", 10, &HectorFivePipesDetection::PclCallback, this);



    ros::NodeHandle pnh("~");
    detection_object_server_.reset(new actionlib::SimpleActionServer<hector_perception_msgs::DetectObjectAction>(pnh, "detect", boost::bind(&HectorFivePipesDetection::executeCallback, this, _1) ,false));
    detection_object_server_->start();

}

HectorFivePipesDetection::~HectorFivePipesDetection()
{}

void HectorFivePipesDetection::PclCallback(const sensor_msgs::PointCloud2& pc_msg){
    ROS_INFO("pcl callback start");
    input_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(pc_msg, *input_cloud);
    ROS_INFO("pcl callback received");
}

void HectorFivePipesDetection::executeCallback(const hector_perception_msgs::DetectObjectGoalConstPtr& goal)
{

    hector_perception_msgs::DetectObjectResult result;


    // Do stuff and set result appropriately
    result.detection_success = findPipes(goal->detect_request.roi_hint.bounding_box_min, goal->detect_request.roi_hint.bounding_box_max, goal->detect_request.roi_hint.header.frame_id);

    detection_object_server_->setSucceeded(result);
}

bool HectorFivePipesDetection::findPipes(const geometry_msgs::Point& min, const geometry_msgs::Point& max, const std::string& frame_id)
{
    // maybe better as service
    // pointcloud from laserscan/ region of intereset in front of the robot ???

    std::cout<<"frame: "<< frame_id<<std::endl;
    ROS_INFO("min x: %f", min.x);
    ROS_INFO("min y: %f", min.y);
    ROS_INFO("min z: %f", min.z);
    ROS_INFO("max x: %f", max.x);
    ROS_INFO("max y: %f", max.y);
    ROS_INFO("max z: %f", max.z);

    bool success = false;

    ros::NodeHandle n("");
  //  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pointcloud_srv_client_ = n.serviceClient<vigir_perception_msgs::PointCloudRegionRequest>("/worldmodel_main/pointcloud_roi");
    vigir_perception_msgs::PointCloudRegionRequest srv;
    vigir_perception_msgs::EnvironmentRegionRequest erreq;
    if(min.x != max.x && min.y != max.y && min.z != max.z && frame_id.empty()){
        //bouding box parameter from action
        erreq.header.frame_id=frame_id;
        erreq.bounding_box_max.x=max.x;
        erreq.bounding_box_max.y=max.y;
        erreq.bounding_box_max.z=max.z;
        erreq.bounding_box_min.x=min.x;
        erreq.bounding_box_min.y=min.y;
        erreq.bounding_box_min.z=min.z;
    }else{
        //default parameter
        erreq.header.frame_id=worldFrame_;
        erreq.bounding_box_max.x=10;
        erreq.bounding_box_max.y=10;
        erreq.bounding_box_max.z=passThroughZMax_;
        erreq.bounding_box_min.x=-10;
        erreq.bounding_box_min.y=-10;
        erreq.bounding_box_min.z=passThroughZMin_;
    }
    erreq.resolution=0;  //0 <=> default
    erreq.request_augment=0;
    srv.request.region_req=erreq;
    srv.request.aggregation_size=500;

    if (input_cloud->empty()){
        ROS_INFO("input cloud data size is 0 // normal for no test");
        if(!pointcloud_srv_client_.call(srv)){
            ROS_ERROR("service: /worldmodel/pointcloud_roi is not working");
            return success;
        }else{
            sensor_msgs::PointCloud2 pointCloud_world;
            pointCloud_world=srv.response.cloud;

            pcl::PCLPointCloud2 pcl_pc;
            pcl_conversions::toPCL(pointCloud_world, pcl_pc);
            pcl::fromPCLPointCloud2(pcl_pc, *input_cloud);
        }
    } else {
        ROS_INFO("input cloud data size is NOT 0 (test setup, cloud was received via callback)");
    }

    input_cloud->header.frame_id=worldFrame_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud_v2(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_plane_seg(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr rest_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    orginal_pub_debug_.publish(input_cloud);

    processCloud_v2=input_cloud;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setOptimizeCoefficients (true);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (planeSegDistTresh_);

    do{
        seg.setInputCloud (processCloud_v2);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate more planar models for the given dataset.");
        }

        if(inliers->indices.size() < numberPointsThresh_){
            break;
        }
        ROS_DEBUG("extract reset points");
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (processCloud_v2);
        extract.setIndices (inliers);
        extract.setNegative(false);
        extract.filter (*output_cloud_plane_seg);
        output_cloud_plane_seg->header.frame_id=worldFrame_;

        extract.setInputCloud (processCloud_v2);
        extract.setIndices (inliers);
        extract.setNegative(true);
        extract.filter (*rest_cloud);
        rest_cloud->header.frame_id=worldFrame_;
        processCloud_v2=rest_cloud;

    }while(1);

    ROS_DEBUG("ouput plane size: %d", (int)output_cloud_plane_seg->size());
    final_cloud_pub_debug_.publish(rest_cloud);

    // clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (rest_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (clusterTolerance_);
    ec.setMinClusterSize (minClusterSize_);
    ec.setMaxClusterSize (maxClusterSize_);
    ec.setSearchMethod (tree);
    ec.setInputCloud (rest_cloud);
    ec.extract (cluster_indices);

    ROS_DEBUG("number cluster: %d", (int)cluster_indices.size());

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    cloud_cluster->header.frame_id=rest_cloud->header.frame_id;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_centers (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_cluster_centers->header.frame_id=rest_cloud->header.frame_id;
    int j = 0;
    double sum_x=0;
    double sum_y=0;
    double sum_z=0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointXYZ center;
        sum_x=0;
        sum_y=0;
        sum_z=0;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            pcl::PointXYZI pushback;
            pushback.x=rest_cloud->points[*pit].x;
            pushback.y=rest_cloud->points[*pit].y;
            pushback.z=rest_cloud->points[*pit].z;
            pushback.intensity=j;
            cloud_cluster->points.push_back (pushback);
            sum_x=sum_x + pushback.x;
            sum_y=sum_y + pushback.y;
            sum_z=sum_z + pushback.z;
        }
        center.x=sum_x/it->indices.size();
        center.y=sum_y/it->indices.size();
        center.z=sum_z/it->indices.size();
        cloud_cluster_centers->points.push_back(center);
        j++;
    }


    cluster_pub_debug_.publish(cloud_cluster);
    cluster_centers_pub_.publish(cloud_cluster_centers);
    // clusters found

 //   if (cloud_cluster_centers->size() != 5){
        ROS_INFO("%i clusters found. Proceed.", cloud_cluster_centers->size());
   //     return success; // false
  //  }

    // get position of clusters / cylinder
    pcl::PointCloud<pcl::PointXYZ>::Ptr start_check_positions (new pcl::PointCloud<pcl::PointXYZ>);
    start_check_positions->header.frame_id=rest_cloud->header.frame_id;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud_cluster_centers);

    std::vector<pcl::PointXYZ> sortedListOfCenters;

    // find mid cluster center
    pcl::PointXYZ centerPoint = pcl::PointXYZ(0, 0, 0);
    for(int i=0; i< cloud_cluster_centers->points.size(); i++){
        pcl::PointXYZ searchPoint;
        centerPoint = pcl::PointXYZ(0, 0, 0);
        searchPoint= cloud_cluster_centers->points.at(i);

        // Neighbors within radius search
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        // if cluster center in the middle of 4 other cluster centers
        if ( kdtree.radiusSearch (searchPoint, searchRadius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 5 )
        {
            ROS_INFO("exactly 5 centers in radius => start check positions found");
            pcl::PointXYZ p;
            sortedListOfCenters.push_back(searchPoint);
            for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
            {
                p=cloud_cluster_centers->points[ pointIdxRadiusSearch[i] ];
                start_check_positions->points.push_back(p);
                centerPoint.x= centerPoint.x + p.x;
                centerPoint.y= centerPoint.y + p.y;
                centerPoint.z= centerPoint.z + p.z;

                // fill sorted list of final centers
                if (p.z > centerPoint.z)
                    sortedListOfCenters.insert(sortedListOfCenters.begin(),p);
                else if (p.z < centerPoint.z)
                    sortedListOfCenters.push_back(p);
                // else: was already put in list.
                }

            centerPoint.x= centerPoint.x / pointIdxRadiusSearch.size ();
            centerPoint.y= centerPoint.y / pointIdxRadiusSearch.size ();
            centerPoint.z= centerPoint.z / pointIdxRadiusSearch.size ();
            success = true;
        }
    }

    if (success){
        // find our orientation of pose
        // make a vector product to find the direction.
        Eigen::Vector3f v0 = Eigen::Vector3f(sortedListOfCenters[0].x - sortedListOfCenters[1].x,
                                                 sortedListOfCenters[0].y - sortedListOfCenters[1].y,
                                                 sortedListOfCenters[0].z - sortedListOfCenters[1].z);
        Eigen::Vector3f v1 = Eigen::Vector3f(sortedListOfCenters[0].x - sortedListOfCenters[2].x,
                                                 sortedListOfCenters[0].y - sortedListOfCenters[2].y,
                                                 sortedListOfCenters[0].z - sortedListOfCenters[2].z);
        Eigen::Vector3f poseOrientation = v0.cross(v1);
        poseOrientation.normalize();
        ROS_INFO("success, poseOrientation x=%f, y=%f, z=%f", poseOrientation[0], poseOrientation[1], poseOrientation[2]);
        // cross product to get pose (this might have to be inverted, so check for robot pose.
        Eigen::Quaternion<float> quat;
        quat = Eigen::AngleAxis<float>(0, poseOrientation);
        ROS_INFO("posequaternion x=%f, y=%f, z=%f, w=%f", quat.x(), quat.y(), quat.z(), quat.w());
        hector_worldmodel_msgs::PosePercept pp;
        pp.header.frame_id= start_check_positions->header.frame_id;
        pp.header.stamp= srv.response.cloud.header.stamp;
        pp.info.class_id= "start_check_pipe";
        pp.info.class_support=1;
        pp.info.object_support=1;
        pp.pose.pose.position.x= centerPoint.x;
        pp.pose.pose.position.y= centerPoint.y;
        pp.pose.pose.position.z= centerPoint.z;
  //    pp.pose.pose.orientation.x= pp.pose.pose.orientation.y = pp.pose.pose.orientation.z= 0;
        pp.pose.pose.orientation.x= quat.x();
        pp.pose.pose.orientation.y= quat.y();
        pp.pose.pose.orientation.z= quat.z();
        pp.pose.pose.orientation.w= 1;

        posePercept_pub_.publish(pp);
        ROS_INFO("PosePercept startcheck postion pipes published");

        five_pipes_pos_pub_.publish(start_check_positions);
    }


    return success;
}

void HectorFivePipesDetection::dynamic_recf_cb(hector_five_pipes_detection::HectorFivePipesDetectionConfig &config, uint32_t level)
{
    passThroughXMin_= config.passThroughXMin;
    passThroughYMin_= config.passThroughYMin;
    passThroughZMin_= config.passThroughZMin;
    passThroughXMax_= config.passThroughXMax;
    passThroughYMax_= config.passThroughYMax;
    passThroughZMax_= config.passThroughZMax;
    voxelGridX_= config.voxelGridX;
    voxelGridY_= config.voxelGridY;
    voxelGridZ_= config.voxelGridZ;
    planeSegDistTresh_= config.planeSegDistTresh;
    numberPointsThresh_= config.numberPointsThresh;
    clusterTolerance_= config.clusterTolerance;
    minClusterSize_= config.minClusterSize;
    maxClusterSize_= config.maxClusterSize;
    searchRadius_= config.searchRadius;

}

}


