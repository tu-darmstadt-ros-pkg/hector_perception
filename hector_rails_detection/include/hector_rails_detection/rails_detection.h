#ifndef HECTOR_RAILS_DETECTION_H
#define HECTOR_RAILS_DETECTION_H

#include <ros/ros.h>


#include <actionlib/server/simple_action_server.h>
#include <hector_perception_msgs/DetectObjectAction.h>
#include <grid_map_msgs/GridMap.h>

namespace hector_rails_detection{

class RailsDetection{

public:
    RailsDetection();
    virtual ~RailsDetection();


    void elevationMapCallback(const grid_map_msgs::GridMap& grid_map);
    void executeCallback(const hector_perception_msgs::DetectObjectGoalConstPtr& goal);


private:
    ros::Subscriber elevation_map_subscriber_;
    boost::shared_ptr<actionlib::SimpleActionServer<hector_perception_msgs::DetectObjectAction> > detection_object_server_;


};
}

#endif //HECTOR_RAILS_DETECTION_H
