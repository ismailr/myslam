#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "myslam_sim_gazebo/simulation.h"

using namespace MYSLAM;

int main (int argc, char** argv)
{
	std::remove ("/home/ism/data/code/rosws/result/finalpose.dat");
	std::remove ("/home/ism/data/code/rosws/result/odom.dat");
	std::remove ("/home/ism/data/code/rosws/result/groundtruth.dat");
	std::remove ("/home/ism/data/code/rosws/result/data.g2o");
	std::remove ("/home/ism/data/code/rosws/result/da.log");

	ros::init (argc,argv,"myslam_sim_gazebo");
	ros::NodeHandle nh;

	Graph g;
	Optimizer o (g);

	MYSLAM::Simulation sim (g, o, nh);

    message_filters::Subscriber<nav_msgs::Odometry> subodom (nh, "odom", 1);
    message_filters::Subscriber<myslam_sim_gazebo::LogicalImage> sublogcam (nh, "logcam", 1);

    typedef message_filters::sync_policies::ApproximateTime 
        <nav_msgs::Odometry, myslam_sim_gazebo::LogicalImage> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync (MySyncPolicy (100), 
        subodom, sublogcam);

    sync.registerCallback (boost::bind (&MYSLAM::Simulation::callback2, &sim, _1, _2/*, _3, _4, _5, _6, _7*/));

    ros::spin();

    return 0;
}
