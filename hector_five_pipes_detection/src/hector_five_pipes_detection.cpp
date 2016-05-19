#include <ros/ros.h>
#include <hector_five_pipes_detection/hector_five_pipes_detection.h>

namespace hector_five_pipes_detection{
HectorFivePipesDetection::HectorFivePipesDetection(){
    ROS_INFO ("HectorFivePipesDetection started");
    ros::NodeHandle nh("");

    //load params
    nh.param("passThroughXMin", passThroughXMin_, 0.1);
    nh.param("passThroughYMin", passThroughYMin_, 0.1);
    nh.param("passThroughZMin", passThroughZMin_, 0.1);
    nh.param("passThroughXMin", passThroughXMax_, 0.1);
    nh.param("passThroughYMin", passThroughYMax_, 0.0);
    nh.param("passThroughZMin", passThroughZMax_, 2.0);
    nh.param("voxelGridX", voxelGridX_, 0.05);
    nh.param("voxelGridY", voxelGridY_, 0.05);
    nh.param("voxelGridZ", voxelGridZ_, 0.05);
    nh.param("planeSegDistTresh", planeSegDistTresh_, 0.03);
    nh.param("numberPointsThresh", numberPointsThresh_, 1000);
    nh.param("minRadius", minRadius_, 0.01);
    nh.param("maxRadius", maxRadius_, 0.1);
    nh.param("clusterTolerance", clusterTolerance_, 0.03);
    nh.param("minClusterSize", minClusterSize_, 10);
    nh.param("maxClusterSize", maxClusterSize_, 1000);


    nh.param("worldFrame", worldFrame_, std::string("/world"));

    orginal_pub_debug_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_five_pipe_detection/input_cloud_debug", 100, true);
    after_pass_through_pub_debug_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_five_pipe_detection/after_pass_through_debug", 100, true);
    after_voxel_grid_pub_debug_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_five_pipe_detection/after_voxel_gird_debug", 100, true);
    final_cloud_pub_debug_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_five_pipe_detection/final_cloud_pub_debug", 100, true);
    plane_pub_debug_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_five_pipe_detection/plane_pub_debug", 100, true);
    cloud_filtered_publisher_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/hector_five_pipe_detection/cylinder_cloud_debug", 100, true);
    cluster_pub_debug_= nh.advertise<pcl::PointCloud<pcl::PointXYZI> >("/hector_five_pipe_detection/cluster_cloud_debug", 100, true);

    pointcloud_sub_ = nh.subscribe("/worldmodel_main/pointcloud_vis", 10, &HectorFivePipesDetection::PclCallback, this);

}

HectorFivePipesDetection::~HectorFivePipesDetection()
{}

// maybe better as service
// pointcloud fomr laserscan/ region of intereset in front of the robot ???
void HectorFivePipesDetection::PclCallback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg){
    // transform frame
    ROS_INFO("stairs position callback enterd");
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*pc_msg,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*input_cloud);
    //transform cloud to /world
    tf::StampedTransform transform_cloud_to_map;
    try{
        ros::Time time = pc_msg->header.stamp;
        listener_.waitForTransform(worldFrame_, pc_msg->header.frame_id,
                                   time, ros::Duration(3.0));
        listener_.lookupTransform(worldFrame_, pc_msg->header.frame_id,
                                  time, transform_cloud_to_map);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("Lookup Transform failed: %s",ex.what());
        return;
    }

    tf::transformTFToEigen(transform_cloud_to_map, to_map_);

    // Transform to /world
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*input_cloud, *cloud_tmp, to_map_);
    input_cloud = cloud_tmp;
    input_cloud->header.frame_id= transform_cloud_to_map.frame_id_;

    // filter pointcloud (pass throught /region of interest, voxelgrid filter)
    ROS_INFO("Hector Stair Detection get Surface");
    pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud_v1(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr processCloud_v2(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_plane_seg(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr rest_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    orginal_pub_debug_.publish(input_cloud);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(passThroughZMin_, passThroughZMax_);
    pass.filter(*processCloud_v2);

    //    pass.setInputCloud(processCloud_v1);
    //    pass.setFilterFieldName("y");
    //    pass.setFilterLimits(passThroughYMin_, passThroughYMax_);
    //    pass.filter(*processCloud_v1);

    //    pass.setInputCloud(processCloud_v1);
    //    pass.setFilterFieldName("x");
    //    pass.setFilterLimits(passThroughXMin_, passThroughXMax_);
    //    pass.filter(*processCloud_v1);

    //    after_pass_through_pub_debug_.publish(processCloud_v1);

    //    pcl::VoxelGrid<pcl::PointXYZ> vox;
    //    vox.setInputCloud(processCloud_v1);
    //    vox.setLeafSize(voxelGridX_, voxelGridY_, voxelGridZ_);
    //    vox.setDownsampleAllData(false);
    //    vox.filter(*processCloud_v2);

    //    after_voxel_grid_pub_debug_.publish(processCloud_v2);

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

    ROS_DEBUG("ouput plane size: %i", output_cloud_plane_seg->size());
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

    ROS_INFO("number cluster: %i", cluster_indices.size());

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    cloud_cluster->header.frame_id=rest_cloud->header.frame_id;
    int j = 0;
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
      {
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
          pcl::PointXYZI pushback;
          pushback.x=rest_cloud->points[*pit].x;
          pushback.y=rest_cloud->points[*pit].y;
          pushback.z=rest_cloud->points[*pit].z;
          pushback.intensity=j;
          cloud_cluster->points.push_back (pushback);
        }
        j++;
      }

      cluster_pub_debug_.publish(cloud_cluster);

    // get position of clusters / cylinder


}

}


