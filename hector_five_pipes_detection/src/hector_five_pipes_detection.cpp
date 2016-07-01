#include <hector_five_pipes_detection/hector_five_pipes_detection.h>

namespace hector_five_pipes_detection{
HectorFivePipesDetection::HectorFivePipesDetection(){
    ROS_DEBUG ("HectorFivePipesDetection started");
    ros::NodeHandle nh("");

    //load params from yaml file else maybe they come from cfg.. ?
    nh.param("x_min_dist_BB", x_min_dist_BB_, 0.1f);
    nh.param("x_max_dist_BB", x_max_dist_BB_, 1.4f);
    nh.param("y_tolarance_BB", y_tolarance_BB_, 0.4f);
    nh.param("z_min_dist_BB", z_min_dist_BB_, 0.1f);
    nh.param("z_max_dist_BB", z_max_dist_BB_, 1.2f);
    nh.param("planeSegDistTresh", planeSegDistTresh_, 0.3f);
    nh.param("numberPointsThresh", numberPointsThresh_, 1000);
    nh.param("clusterTolerance", clusterTolerance_, 0.03f);
    nh.param("minClusterSize", minClusterSize_, 10);
    nh.param("maxClusterSize", maxClusterSize_, 1000);
    nh.param("searchRadius", searchRadius_, 0.2f);
    nh.param("filter_radius", filter_radius_, 0.03f);
    nh.param("filter_cloud_n_neighbors", filter_cloud_n_neighbors_, 50);
    nh.param("filter_cloud_max_stdev", filter_cloud_max_stdev_, 1.0f);

    worldFrame_= std::string("/world");

    dynamic_recf_type = boost::bind(&HectorFivePipesDetection::dynamic_recf_cb, this, _1, _2);
    dynamic_recf_server.setCallback(dynamic_recf_type);

    orginal_pub_debug_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_five_pipe_detection/input_cloud_debug", 100, true);
    filtered_cloud_debug_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_five_pipe_detection/filtered_cloud_debug", 100, true);
    roi_debug_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_five_pipe_detection/pcl_roi", 100, true);
    after_voxel_grid_pub_debug_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_five_pipe_detection/after_voxel_gird_debug", 100, true);
    cloud_without_planes_pub_debug_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_five_pipe_detection/final_cloud_pub_debug", 100, true);
    plane_pub_debug_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_five_pipe_detection/plane_pub_debug", 100, true);
    cloud_filtered_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_five_pipe_detection/cylinder_cloud_debug", 100, true);
    cluster_pub_debug_= nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("/hector_five_pipe_detection/cluster_cloud_debug", 100, true);
    five_pipes_pos_pub_= nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("/hector_five_pipe_detection/five_pipes_positions", 100, true);
    cluster_centers_pub_= nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("/hector_five_pipe_detection/cloud_centers", 100, true);
    posePercept_pub_= nh.advertise<hector_worldmodel_msgs::PosePercept>("/worldmodel/pose_percept", 0);
    posePercept_debug_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/hector_five_pipe_detection/pose_debug", 0); // oder posePercept stamped
    endPoseDebugPCL_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_five_pipe_detection/endPoseDebugOrientation", 100, true);

    pointcloud_sub_ = nh.subscribe("/arm_rgbd_cam/depth/points", 10, &HectorFivePipesDetection::PclCallback, this);

    ros::NodeHandle pnh("~");
    detection_object_server_.reset(new actionlib::SimpleActionServer<hector_perception_msgs::DetectObjectAction>(pnh, "detect", boost::bind(&HectorFivePipesDetection::executeCallback, this, _1) ,false));
    detection_object_server_->start();

    robot_pose_init = false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr first_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    input_cloud = first_cloud;

}

HectorFivePipesDetection::~HectorFivePipesDetection()
{}

void HectorFivePipesDetection::PclCallback(const sensor_msgs::PointCloud2& pc_msg){

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pcl(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(pc_msg, *current_pcl);
    if(!current_pcl->empty()){
        input_cloud=current_pcl;
    }
    else{
        ROS_INFO("camera pcl is empty keeping latest non empty cloud for pipe detection");
    }
}

void HectorFivePipesDetection::executeCallback(const hector_perception_msgs::DetectObjectGoalConstPtr& goal)
{
    hector_perception_msgs::DetectObjectResult result;
    result.detection_success = findPipes(goal->detect_request.roi_hint.bounding_box_min, goal->detect_request.roi_hint.bounding_box_max, goal->detect_request.roi_hint.header.frame_id);
    detection_object_server_->setSucceeded(result);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr HectorFivePipesDetection::cleanPointCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_withouth_nans(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < pointCloud->size(); i++){
        pcl::PointXYZ p = pointCloud->at(i);
        if (p.x == p.x && p.y == p.y && p.z == p.z){
            cloud_withouth_nans->push_back(p);
        }
    }
    cloud_withouth_nans->header.frame_id = pointCloud->header.frame_id;
    cloud_withouth_nans->header.stamp = pointCloud->header.stamp;
    return cloud_withouth_nans;
}

bool HectorFivePipesDetection::findPipes(const geometry_msgs::Point& min, const geometry_msgs::Point& max, const std::string& frame_id)
{
    // maybe better as service
    std::cout<<"frame: "<< frame_id<<std::endl;
    bool success = false;

    ros::NodeHandle n("");
  //  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //pointcloud_srv_client_ = n.serviceClient<vigir_perception_msgs::PointCloudRegionRequest>("/worldmodel_main/pointcloud_roi");
//    vigir_perception_msgs::PointCloudRegionRequest srv;
//    vigir_perception_msgs::EnvironmentRegionRequest erreq;
//    if(min.x != max.x && min.y != max.y && min.z != max.z && frame_id.empty()){
//        //bouding box parameter from action
//        erreq.header.frame_id=frame_id;
//        erreq.bounding_box_max.x=max.x;
//        erreq.bounding_box_max.y=max.y;
//        erreq.bounding_box_max.z=max.z;
//        erreq.bounding_box_min.x=min.x;
//        erreq.bounding_box_min.y=min.y;
//        erreq.bounding_box_min.z=min.z;
//    }else{
//        //default parameter
//        erreq.header.frame_id=worldFrame_;
//        erreq.bounding_box_max.x=10;
//        erreq.bounding_box_max.y=10;
//        erreq.bounding_box_max.z=passThroughZMax_;
//        erreq.bounding_box_min.x=-10;
//        erreq.bounding_box_min.y=-10;
//        erreq.bounding_box_min.z=passThroughZMin_;
//    }
//    erreq.resolution=0;  //0 <=> default
//    erreq.request_augment=0;
//    srv.request.region_req=erreq;
//    srv.request.aggregation_size=500;
    if (input_cloud->empty()){
    //    ROS_INFO("input cloud data size is 0 // normal for no test");
   //     if(!pointcloud_srv_client_.call(srv)){
            ROS_ERROR("input cloud is empty (no data from /arm_rgbd_cam/depth/points reveived");
            return success;
        }
    input_cloud = cleanPointCloud(input_cloud);

    ROS_INFO("transforming cloud");
    tf::StampedTransform transform_cloud_to_world;
    try{
        ros::Time time = ros::Time(0);
        tf_listener.waitForTransform(worldFrame_, input_cloud->header.frame_id,
                                      time, ros::Duration(2.0));
        tf_listener.lookupTransform(worldFrame_, input_cloud->header.frame_id,
                                     time, transform_cloud_to_world);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("Lookup Transform failed: %s",ex.what());
        return false;
    }

    // filter the cloud;
    bool filtercloud = false;
    if (filter_cloud_max_stdev_ > 0.00001 && filtercloud){ // do not use filter if stdev == 0
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (input_cloud);
        sor.setMeanK (filter_cloud_n_neighbors_);
        sor.setStddevMulThresh (filter_cloud_max_stdev_);
        sor.filter (*cloud_filtered);
        ROS_INFO("cloud filtered from size %i to %i. n_neighbors = %i and std = %f", input_cloud->size(), cloud_filtered->size(), filter_cloud_n_neighbors_, filter_cloud_max_stdev_);
        input_cloud = cloud_filtered;
        filtered_cloud_debug_.publish(input_cloud);
    }


    Eigen::Affine3d to_world_;
    tf::transformTFToEigen(transform_cloud_to_world, to_world_);
    // Transform to /darias
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*input_cloud, *cloud_tmp, to_world_);
    input_cloud = cloud_tmp;
    input_cloud->header.frame_id= transform_cloud_to_world.frame_id_;

    ROS_DEBUG("done transforming, input cloud size = %i", input_cloud->size());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud_v2(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_plane_seg(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_planes(new pcl::PointCloud<pcl::PointXYZ>());

    orginal_pub_debug_.publish(input_cloud);
    for (int i = 0; i < input_cloud->size(); i++){
        pcl::PointXYZ p = input_cloud->at(i);
        cloud_roi->push_back(p);
    }

    ROS_DEBUG("roi_cloud computed. size = %i", cloud_roi->size());

    tf::StampedTransform transform;
    ROS_DEBUG("try tf listener find transform");
    try{
      tf_listener.lookupTransform("/world", "/base_link", ros::Time(0), transform);
      robot_pose_init = true;
    }catch(tf::TransformException e){
      ROS_ERROR("Transform lookup failed in get 5pipes detection server goal callback: %s",e.what());
    }

    // use ROI = region of interest in front of the robot
    bool useROI = false;
    if (robot_pose_init && useROI){
        robot_rotation.x() = transform.getRotation().x();
        robot_rotation.y() = transform.getRotation().y();
        robot_rotation.z() = transform.getRotation().z();
        robot_rotation.w() = transform.getRotation().w();
        robot_position[0] = transform.getOrigin().x();
        robot_position[1] = transform.getOrigin().y();
        robot_position[2] = transform.getOrigin().z();

        cloud_roi->clear();
        // roboter pose
        ROS_INFO("5pipes: robot at x=%f, y=%f, z=%f", robot_position[0], robot_position[1], robot_position[2]);
        Eigen::Quaternionf q = robot_rotation;
        q.normalize();
        Eigen::Vector3f xAxis(1,0,0);
        Eigen::Quaternionf p;
        p.w() = 0;
        p.vec() = xAxis;
        Eigen::Quaternionf rotatedP = q * p * q.inverse();
        Eigen::Vector3f robot_x_axis = rotatedP.vec();
        robot_x_axis[2] = 0;
        robot_x_axis.normalize();
        Eigen::Vector3f robot_y_axis = Eigen::Vector3f(robot_x_axis[1], -1*robot_x_axis[0], 0);
        ROS_INFO("5pipes: robot orientation towards x=%f, y=%f, z=%f", robot_x_axis[0], robot_x_axis[1], robot_x_axis[2]);
        // region of interest from robot position
        //float radius = 0.5;
        //float center_x = robot_position[0] + robot_x_axis[0]*center_dist_x;
        //float center_y = robot_position[1] + robot_x_axis[1]*center_dist_x;

        float BB_center_x = robot_x_axis[0]*(x_max_dist_BB_-x_min_dist_BB_) + robot_x_axis[1]*0;
        float BB_center_y = robot_y_axis[0]*(x_max_dist_BB_-x_min_dist_BB_) + robot_y_axis[1]*0;
        float BB_center_z = z_max_dist_BB_ - z_min_dist_BB_+ robot_position[2];
        ROS_INFO("5pipes: center of roi BB x=%f, y=%f, z=%f", BB_center_x, BB_center_y, BB_center_z);
        for (int i = 0; i < input_cloud->size(); i++){
            pcl::PointXYZ p = input_cloud->at(i);
            float x = p.x; float y = p.y; float z = p.z;
            // using scalar product
            float x_dist = robot_x_axis[0]*x + robot_x_axis[1]*y; // using scalar product
            bool inx = x_dist > x_min_dist_BB_ && x_dist < x_max_dist_BB_;
            float y_dist = std::abs(robot_y_axis[0]*x + robot_y_axis[1]*y);
            bool iny = y_dist < y_tolarance_BB_;
            float z_dist = z - robot_position[2];
            bool inz = z_dist > z_min_dist_BB_ && z_dist < z_max_dist_BB_;
            if (inx && iny && inz){
                cloud_roi->push_back(p);
            }


            // circular box start
            /*pcl::PointXYZ p = input_cloud->at(i);
            float x = p.data[0];
            float y = p.data[1];
            float z = p.data[2];
            float dist = std::sqrt(std::pow(center_x-x,2) + std::pow(center_y-y,2) + std::pow(center_z-z,2));
            if (dist < radius && dist > 0){
                cloud_roi->push_back(p);
            }*/ // this is for circular box end
        }
        ROS_INFO("roi_cloud computed. size = %i", cloud_roi->size());
    }

    cloud_roi->header.frame_id = input_cloud->header.frame_id;
    cloud_roi->header.stamp = input_cloud->header.stamp;
    roi_debug_pub_.publish(cloud_roi);



    //TODO from here on cloud_roi should be used instead of input cloud
    if (cloud_roi->size() > 100){
        processCloud_v2 = cloud_roi;
        ROS_INFO("using ROI cloud");
    }
    else{
        processCloud_v2=input_cloud;
        ROS_INFO("ROI cloud %i too small, using input cloud instead, size = %i", cloud_roi->size(), input_cloud->size());
    }
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setOptimizeCoefficients (true);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (planeSegDistTresh_);

    // work on processCloud_v2
    bool plane_segmentation = false; // false = only ground away
    if (plane_segmentation){
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
            extract.filter (*cloud_without_planes);
            cloud_without_planes->header.frame_id=worldFrame_;
            processCloud_v2=cloud_without_planes;

        }while(1);
    }
    else {
        ROS_INFO("planesegmentation only ground");
        // make histogramm of z values around 0;
        std::vector<float> bagvalues;
        std::vector<int> bags;
        int container = 7;
        float min = processCloud_v2->at(0).z;
        for (int i = 0; i < processCloud_v2->size(); i++){
            pcl::PointXYZ p = processCloud_v2->at(i);
            float z = p.z;
            z < min;
            min = z;
        }
        float dist = 0.05; // bagsize 5 cm
        for (int i = 0; i = container; i++){
            bagvalues.push_back(min + i*dist);
            bags.push_back(0);
        }
        //fill bags
        for (int i = 0; i < processCloud_v2->size(); i++){
            pcl::PointXYZ p = processCloud_v2->at(i);
            float z = p.z;
            for (int k = 0; k < bagvalues.size() - 1; k++)
                if (z>bagvalues[k] && z <=bagvalues[k+1])
                    bags[k]++;

        }
        int maxidx = 0;
        for (int i = 0; i < bags.size()-1; i++){
            if (bags[i] > bags[maxidx])
                maxidx = i;
        }
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>());        
        //remove the points from cloud
        for (int i = 0; i < processCloud_v2->size(); i++){
            pcl::PointXYZ p = processCloud_v2->at(i);
            float z = p.z;
            if (z < bagvalues[maxidx] || z >bagvalues[maxidx+1])
                temp->push_back(p);
        }
        processCloud_v2 = temp;
    }

    ROS_INFO("ouput plane size: %d", (int)output_cloud_plane_seg->size());
    cloud_without_planes_pub_debug_.publish(cloud_without_planes);

    // clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    cloud_without_planes = cleanPointCloud(cloud_without_planes);
    tree->setInputCloud (cloud_without_planes);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (clusterTolerance_);
    ec.setMinClusterSize (minClusterSize_);
    ec.setMaxClusterSize (maxClusterSize_);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_without_planes);
    ec.extract (cluster_indices);

    ROS_DEBUG("number cluster: %d", (int)cluster_indices.size());

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    cloud_cluster->header.frame_id=cloud_without_planes->header.frame_id;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_centers (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_cluster_centers->header.frame_id=cloud_without_planes->header.frame_id;
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
            pushback.x=cloud_without_planes->points[*pit].x;
            pushback.y=cloud_without_planes->points[*pit].y;
            pushback.z=cloud_without_planes->points[*pit].z;
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

    ROS_INFO("5pipes: %i clusters found. Proceed.", cloud_cluster_centers->size());

    // filter clusters
    bool filter_clusters_active = false; //PARAM
    if (filter_clusters_active){
        ROS_INFO("enter filtering clusters");
        float max_radius = 0.03;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_centers_copy (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_centers_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        for(int i=0; i< cloud_cluster_centers->points.size(); i++){
            pcl::PointXYZ p = cloud_cluster_centers->points.at(i);
            cloud_cluster_centers_copy->push_back(p);
        }

        while (cloud_cluster_centers_copy->empty() == false){
            pcl::PointXYZ p1 = cloud_cluster_centers_copy->front();
            float n = 0;
            float sumx = 0.0;
            float sumy = 0.0;
            float sumz = 0.0;
            int i = 0;
            while (cloud_cluster_centers_copy->empty() == false){
                pcl::PointXYZ p2 = cloud_cluster_centers_copy->front();
                float dist = std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y,2) + std::pow(p1.z - p2.z,2));
                if (dist < max_radius){
                    n = n+1;
                    sumx = p2.x;
                    sumy = p2.y;
                    sumz = p2.z;
                    cloud_cluster_centers_copy->erase(cloud_cluster_centers_copy->begin()+i);
                }
                else{
                    i++;
                }
            }
            pcl::PointXYZ newP;
            newP.x = sumx / n;
            newP.y = sumy / n;
            newP.z = sumz / n;
            cluster_centers_filtered->push_back(newP);
        }
        ROS_DEBUG("end filtering clusters, size = %i", cluster_centers_filtered->size());
    }
    ROS_INFO("cluster_centers_filtered unused");
    // TODO use the cloud processed


    // get position of clusters / cylinder
    pcl::PointCloud<pcl::PointXYZ>::Ptr start_check_positions (new pcl::PointCloud<pcl::PointXYZ>);
    start_check_positions->header.frame_id=cloud_without_planes->header.frame_id;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud_cluster_centers);

    std::vector<pcl::PointXYZ> sortedListOfCenters;

    int min_cluster_centers_start_pose = 5; // PARAM
    // find mid cluster center
    pcl::PointXYZ centerPoint = pcl::PointXYZ(0, 0, 0);
    for(int i=0; i< cloud_cluster_centers->points.size(); i++){
        pcl::PointXYZ searchPoint= cloud_cluster_centers->points.at(i);

        // Neighbors within radius search
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        // if cluster center in the middle of 4 other cluster centers
        if ( kdtree.radiusSearch (searchPoint, searchRadius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) == min_cluster_centers_start_pose )
        {
            ROS_INFO("5pipes: %i or more centers in radius => start check positions found", min_cluster_centers_start_pose);
            pcl::PointXYZ p;
            sortedListOfCenters.push_back(searchPoint);
            for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
                p=cloud_cluster_centers->points[ pointIdxRadiusSearch[i] ];
                start_check_positions->points.push_back(p);
                centerPoint.x= centerPoint.x + p.x;
                centerPoint.y= centerPoint.y + p.y;
                centerPoint.z= centerPoint.z + p.z;

                // fill sorted list of final centers // !!! NOT SORTED!!!!!
                if (p.z > centerPoint.z)
                    sortedListOfCenters.insert(sortedListOfCenters.begin(),p);
                else if (p.z < centerPoint.z)
                    sortedListOfCenters.push_back(p);
                // else: was already put in list.
            }
            ROS_DEBUG("pointssize should be 5 : %i", pointIdxRadiusSearch.size());
            centerPoint.x= centerPoint.x / pointIdxRadiusSearch.size ();
            centerPoint.y= centerPoint.y / pointIdxRadiusSearch.size ();
            centerPoint.z= centerPoint.z / pointIdxRadiusSearch.size ();
            success = true;
            break;
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
        float roll = 0;
        float pitch = atan2(poseOrientation[2], poseOrientation[0]); //y
        float yaw = atan2(poseOrientation[1], poseOrientation[0]); //z
                //acos(poseOrientation.dot(Eigen::Vector3f::UnitZ()));
        ROS_INFO("for quaternion: yawAngle = %f", yaw);
        Eigen::AngleAxisf rollTransform(roll, Eigen::Vector3f::UnitX()); // should do nothing
        Eigen::AngleAxisf yawTransform(yaw, Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf pitchTransform(pitch, Eigen::Vector3f::UnitY());

        Eigen::Quaternionf quat1 = Eigen::Quaternionf(yawTransform);
        Eigen::Quaternionf quat2 = Eigen::Quaternionf(pitchTransform);
        Eigen::Quaternionf quat = quat1 *quat2;
        ROS_DEBUG("posequaternion x=%f, y=%f, z=%f, w=%f", quat.x(), quat.y(), quat.z(), quat.w());
        ROS_DEBUG("pose x=%f, y=%f, z=%f", centerPoint.x, centerPoint.y, centerPoint.z);
        hector_worldmodel_msgs::PosePercept pp;
        pp.header.frame_id= start_check_positions->header.frame_id;
        pp.header.stamp= ros::Time(0);
        pp.info.class_id= "pipes";
        pp.info.class_support=1;
        pp.info.object_support=1;
        pp.pose.pose.position.x= centerPoint.x;
        pp.pose.pose.position.y= centerPoint.y;
        pp.pose.pose.position.z= centerPoint.z;
        pp.pose.pose.orientation.x= quat.x();
        pp.pose.pose.orientation.y= quat.y();
        pp.pose.pose.orientation.z= quat.z();
        pp.pose.pose.orientation.w= quat.w();
        std::vector <float>flatpointarray; // watch out for changing size of n pipes!!
        for (int i = 0; i < sortedListOfCenters.size(); i++){
            pcl::PointXYZ p = sortedListOfCenters[i];
            flatpointarray.push_back(p.x);
            flatpointarray.push_back(p.y);
            flatpointarray.push_back(p.z);
        }
        pp.info.data = flatpointarray;

        geometry_msgs::PoseStamped pose_debug;
        pose_debug.header.frame_id = pp.header.frame_id;
        pose_debug.header.stamp = pp.header.stamp;
        pose_debug.pose.orientation.w = quat.w();
        pose_debug.pose.orientation.x = quat.x();
        pose_debug.pose.orientation.y = quat.y();
        pose_debug.pose.orientation.z = quat.z();
        pose_debug.pose.position.x = centerPoint.x;
        pose_debug.pose.position.y = centerPoint.y;
        pose_debug.pose.position.z = centerPoint.z;
        posePercept_pub_.publish(pp);
        posePercept_debug_pub_.publish(pose_debug);
        ROS_INFO("PosePercept startcheck postion pipes published");

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_orientation(new pcl::PointCloud<pcl::PointXYZ>());
        cloud_orientation->header.frame_id = worldFrame_;
        cloud_orientation->header.stamp = input_cloud->header.stamp;
        float l = 0.1;
        int n = 20;
        for (int i = 0; i < n; i++){
            cloud_orientation->push_back(pcl::PointXYZ(centerPoint.x + i*l*poseOrientation[0], centerPoint.y + i*l*poseOrientation[1], centerPoint.z + i*l*poseOrientation[2]));
        }
        endPoseDebugPCL_.publish(cloud_orientation);

        five_pipes_pos_pub_.publish(start_check_positions);
    }
    else {
        ROS_WARN("no radius search with %i (or more?) cluster centers found.", min_cluster_centers_start_pose);
    }


    return success;
}

void HectorFivePipesDetection::dynamic_recf_cb(hector_five_pipes_detection::HectorFivePipesDetectionConfig &config, uint32_t level)
{
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
    filter_radius_ = config.filter_radius;
    filter_cloud_n_neighbors_ = config.filter_cloud_n_neighbors;
    filter_cloud_max_stdev_ = config.filter_cloud_max_stdev;

}

}


