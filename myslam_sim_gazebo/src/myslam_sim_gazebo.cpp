#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "myslam_sim_gazebo/LogicalImage.h"
#include "myslam_sim_gazebo/Model.h"

void callback (const nav_msgs::Odometry::ConstPtr& odom,
        const myslam_sim_gazebo::LogicalImage::ConstPtr& logimg)
{
}

int main (int argc, char** argv)
{
	ros::init (argc,argv,"myslam_sim_gazebo");
	ros::NodeHandle nh;

    message_filters::Subscriber<nav_msgs::Odometry> subodom (nh, "odom", 1);
    message_filters::Subscriber<myslam_sim_gazebo::LogicalImage> sublogcam (nh, "logcam", 1);

    typedef message_filters::sync_policies::ApproximateTime 
        <nav_msgs::Odometry, myslam_sim_gazebo::LogicalImage> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync (MySyncPolicy (100), 
        subodom, sublogcam);

    sync.registerCallback (boost::bind (&callback, _1, _2/*, _3, _4, _5, _6, _7*/));

    ros::spin();

    return 0;
}
