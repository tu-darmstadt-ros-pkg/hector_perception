#include <hector_five_pipes_detection/hector_five_pipes_detection.h>

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

    pointcloud_sub_ = nh.subscribe("/worldmodel_main/pointcloud_vis", 10, &HectorFivePipesDetection::PclCallback, this);
//    tf_sub_ = nh.subscribe("/tf", 10, &HectorFivePipesDetection::TfCallback, this);

    ros::NodeHandle pnh("~");
    detection_object_server_.reset(new actionlib::SimpleActionServer<hector_perception_msgs::DetectObjectAction>(pnh, "detect", boost::bind(&HectorFivePipesDetection::executeCallback, this, _1) ,false));
    detection_object_server_->start();

    robot_pose_init = false;

}

HectorFivePipesDetection::~HectorFivePipesDetection()
{}

/*
void HectorFivePipesDetection::TfCallback(const tf2_msgs::TFMessage &tf_msg){
    std::vector<geometry_msgs::TransformStamped> tf_transform_stamped_vec = tf_msg.transforms;
    geometry_msgs::TransformStamped robot_pose;
    std::string baselink = "base_link";
    for (int i = 0; i < tf_transform_stamped_vec.size(); i++){
        geometry_msgs::TransformStamped tfs = tf_transform_stamped_vec[i];
        std::string child_frame_id = tfs.child_frame_id;
        if (baselink.compare(child_frame_id)){
            ROS_INFO("child_frame_id = %s, frame_id = %s", child_frame_id.);
            robot_pose = tf_transform_stamped_vec[i];
            robot_rotation.x() = robot_pose.transform.rotation.x;
            robot_rotation.y() = robot_pose.transform.rotation.y;
            robot_rotation.z() = robot_pose.transform.rotation.z;
            robot_rotation.w() = robot_pose.transform.rotation.w;
            robot_position[0] = robot_pose.transform.translation.x;
            robot_position[1] = robot_pose.transform.translation.y;
            robot_position[2] = robot_pose.transform.translation.z;
            robot_pose_init = true;
            // robot pose found via frame_id_comparison
            break;
        }
    }
}*/


void HectorFivePipesDetection::PclCallback(const sensor_msgs::PointCloud2& pc_msg){
    input_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(pc_msg, *input_cloud);
    //ROS_INFO("5pipesDetection: pcl callback received, Robot pose init = %i", robot_pose_init);
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
 /*   ROS_INFO("min x: %f", min.x);
    ROS_INFO("min y: %f", min.y);
    ROS_INFO("min z: %f", min.z);
    ROS_INFO("max x: %f", max.x);
    ROS_INFO("max y: %f", max.y);
    ROS_INFO("max z: %f", max.z);*/

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud_v2(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_plane_seg(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_withouth_planes(new pcl::PointCloud<pcl::PointXYZ>());

    orginal_pub_debug_.publish(input_cloud);
    for (int i = 0; i < input_cloud->size(); i++){
        pcl::PointXYZ p = input_cloud->at(i);
        cloud_roi->push_back(p);
    }
    //cloud_roi = input_cloud;
    ROS_INFO("roi_cloud computed. size = %i", cloud_roi->size());
    int count = 0;

    tf::StampedTransform transform;
    ROS_INFO("try tf listener find transform");
    try{
      tf_listener.lookupTransform("/world", "/base_link", ros::Time(0), transform);
      robot_pose_init = true;
    }catch(tf::TransformException e){
      ROS_INFO("failed tf listener find transform");
      ROS_ERROR("Transform lookup failed in get 5pipes detection server goal callback: %s",e.what());
    }

    robot_pose_init = false;
    if (robot_pose_init){

        // TODO use transform
        robot_rotation.x() = transform.getRotation().x();
        robot_rotation.y() = transform.getRotation().y();
        robot_rotation.z() = transform.getRotation().z();
        robot_rotation.w() = transform.getRotation().w();
        robot_position[0] = transform.getOrigin().x();
        robot_position[1] = transform.getOrigin().y();
        robot_position[2] = transform.getOrigin().z();

        cloud_roi->clear();
        // roboter pose
        ROS_INFO("5pipes() started, robot x=%f, y=%f, z=%f", robot_position[0], robot_position[1], robot_position[2]);
        Eigen::Quaternionf q = robot_rotation;
        q.normalize();
        Eigen::Vector3f xAxis(1,0,0);
        Eigen::Quaternionf p;
        p.w() = 0;
        p.vec() = xAxis;
        Eigen::Quaternionf rotatedP = q * p * q.inverse();
        Eigen::Vector3f robot_x_axis = rotatedP.vec();
        ROS_INFO("robot points towards x=%f, y=%f, z=%f", robot_x_axis[0], robot_x_axis[1], robot_x_axis[2]);
        // region of interest from robot position
        /*float min_dist_x = 0.2;
        float max_dist_x = 1.5;
        float min_dist_y = -0.5;
        float max_dist_y = 0.5;
        float min_z = 0;
        float max_z = 1.5;*/

        // above TODO, first try out with circular bounding box
        float center_dist_x = 0.7;
        // float center_dist_y = 0; // unused because 0.
        float center_z = 0.7;
        float radius = 0.5;

        float center_x = robot_position[0] + robot_x_axis[0]*center_dist_x;
        float center_y = robot_position[1] + robot_x_axis[1]*center_dist_x;
        pcl::PointXYZ center = pcl::PointXYZ(center_x, center_y, center_z);
        ROS_INFO("center of roi x=%f, y=%f, z=%f", center_x, center_y, center_z);

        for (int i = 0; i < input_cloud->size(); i++){
            pcl::PointXYZ p = input_cloud->at(i);
            float x = p.data[0];
            float y = p.data[1];
            float z = p.data[2];
            float dist = std::sqrt(std::pow(center_x-x,2) + std::pow(center_y-y,2) + std::pow(center_z-z,2));
            if (dist < radius){
                count++;
                cloud_roi->push_back(p);
            }
        }
        ROS_INFO("roi_cloud computed. size = %i", cloud_roi->size());
    }

    cloud_roi->header.frame_id = input_cloud->header.frame_id;
    cloud_roi->header.stamp = input_cloud->header.stamp;
    roi_debug_pub_.publish(cloud_roi);



    //TODO from here on cloud_roi should be used instead of input cloud
    processCloud_v2=cloud_roi;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setOptimizeCoefficients (true);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (planeSegDistTresh_);

    // plane segmentation
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
        extract.filter (*cloud_withouth_planes);
        cloud_withouth_planes->header.frame_id=worldFrame_;
        processCloud_v2=cloud_withouth_planes;

    }while(1);

    ROS_DEBUG("ouput plane size: %d", (int)output_cloud_plane_seg->size());
    cloud_without_planes_pub_debug_.publish(cloud_withouth_planes);

    // clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_withouth_planes);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (clusterTolerance_);
    ec.setMinClusterSize (minClusterSize_);
    ec.setMaxClusterSize (maxClusterSize_);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_withouth_planes);
    ec.extract (cluster_indices);

    ROS_DEBUG("number cluster: %d", (int)cluster_indices.size());

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    cloud_cluster->header.frame_id=cloud_withouth_planes->header.frame_id;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_centers (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_cluster_centers->header.frame_id=cloud_withouth_planes->header.frame_id;
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
            pushback.x=cloud_withouth_planes->points[*pit].x;
            pushback.y=cloud_withouth_planes->points[*pit].y;
            pushback.z=cloud_withouth_planes->points[*pit].z;
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

    // filter clusters
 /*   bool filter_clusters_active = false;
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
    // TODO use the cloud processed

*/

    // get position of clusters / cylinder
    pcl::PointCloud<pcl::PointXYZ>::Ptr start_check_positions (new pcl::PointCloud<pcl::PointXYZ>);
    start_check_positions->header.frame_id=cloud_withouth_planes->header.frame_id;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud_cluster_centers);

    std::vector<pcl::PointXYZ> sortedListOfCenters;

    // find mid cluster center
    pcl::PointXYZ centerPoint = pcl::PointXYZ(0, 0, 0);
    for(int i=0; i< cloud_cluster_centers->points.size(); i++){
        pcl::PointXYZ searchPoint= cloud_cluster_centers->points.at(i);

        // Neighbors within radius search
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        // if cluster center in the middle of 4 other cluster centers
        if ( kdtree.radiusSearch (searchPoint, searchRadius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 3 )
        {
            ROS_INFO("4 or more centers in radius => start check positions found");
            pcl::PointXYZ p;
            sortedListOfCenters.push_back(searchPoint);
            for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
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
        // TODO calc roll pitch yaw (take into account that plane is already parallel to Z and generate quaternion from it)
        ROS_INFO("success, poseOrientation x=%f, y=%f, z=%f", poseOrientation[0], poseOrientation[1], poseOrientation[2]);
        // cross product to get pose (this might have to be inverted, so check for robot pose.
        // TODO atan2(y,x)
        //Eigen::Vector3f axis(1,1,1);
        float roll = 0;
        float pitch = acos(poseOrientation.dot(Eigen::Vector3f::UnitZ()));
        float yaw = 0;
        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitX());

        Eigen::Quaternion<float> quat = rollAngle * yawAngle * pitchAngle;
       // Eigen::Quaternion<float> quat = Eigen::Quaternion<float>::FromTwoVectors(axis, poseOrientation);
        //quat = Eigen::AngleAxis<float>(0, poseOrientation);
        ROS_DEBUG("posequaternion x=%f, y=%f, z=%f, w=%f", quat.x(), quat.y(), quat.z(), quat.w());
        ROS_DEBUG("pose x=%f, y=%f, z=%f", centerPoint.x, centerPoint.y, centerPoint.z);
        hector_worldmodel_msgs::PosePercept pp;
        pp.header.frame_id= start_check_positions->header.frame_id;
        pp.header.stamp= srv.response.cloud.header.stamp;
        pp.info.class_id= "pipes";
        pp.info.class_support=1;
        pp.info.object_support=1;
        pp.pose.pose.position.x= centerPoint.x;
        pp.pose.pose.position.y= centerPoint.y;
        pp.pose.pose.position.z= centerPoint.z;
  //    pp.pose.pose.orientation.x= pp.pose.pose.orientation.y = pp.pose.pose.orientation.z= 0;
        pp.pose.pose.orientation.x= quat.x();
        pp.pose.pose.orientation.y= quat.y();
        pp.pose.pose.orientation.z= quat.z();
        pp.pose.pose.orientation.w= quat.w();

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

        five_pipes_pos_pub_.publish(start_check_positions);
    }
    else {
        ROS_WARN("no radius search with 5 cluster centers found.");
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


