#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>

#include "layout_prediction/settings.h"
#include "layout_prediction/helpers.h"
#include "layout_prediction/wall.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/tracker.h"
#include "layout_prediction/wall_detector.h"
#include "layout_prediction/system.h"
#include "layout_prediction/graph.h"
#include "layout_prediction/optimizer.h"

#include <thread>

int main (int argc, char** argv)
{
	pcl::console::setVerbosityLevel (pcl::console::L_ALWAYS);

	ros::init (argc,argv,"layout_prediction");
	ros::NodeHandle nh;

    // Initialize system
    System *system = new System (nh);
    Tracker *tracker = new Tracker ();
    WallDetector *wall_detector = new WallDetector ();
    Graph *graph = new Graph ();
    Optimizer *optimizer = new Optimizer (graph);
    
    system->setTracker (tracker);
    system->setWallDetector (wall_detector);

    // initialize threads
    std::thread wall_detector_thread (&WallDetector::run, wall_detector);
    std::thread optimizer_thread (&Optimizer::run, optimizer);

    // Get action data
	ros::Subscriber sub_odom = nh.subscribe ("action", 1, &System::readActionData, system);

    // Get sensors data
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub (nh, "cloud", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub (nh, "depth", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odometry_sub (nh, "odometry", 1);

    typedef message_filters::sync_policies::ApproximateTime
//        <sensor_msgs::PointCloud2, sensor_msgs::Image, nav_msgs::Odometry> MySyncPolicy;
//    message_filters::Synchronizer<MySyncPolicy> sync (MySyncPolicy (10), cloud_sub, depth_sub, odometry_sub);
//    sync.registerCallback (boost::bind (&System::readSensorsData, system, _1, _2, _3));
        <sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync (MySyncPolicy (10), cloud_sub, odometry_sub);
    sync.registerCallback (boost::bind (&System::readSensorsData, system, _1, _2));


    ros::spin();
//    ros::Rate r (10);
//    while (ros::ok())
//    {
//        ros::spinOnce ();
//        r.sleep ();
//    }

	return 0;
}

