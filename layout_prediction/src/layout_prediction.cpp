#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>

#include "layout_prediction/settings.h"
#include "layout_prediction/helpers.h"
#include "layout_prediction/wall_detector.h"
#include "layout_prediction/system.h"
#include "layout_prediction/optimizer.h"

#include <thread>

int main (int argc, char** argv)
{
	pcl::console::setVerbosityLevel (pcl::console::L_ALWAYS);

	ros::init (argc,argv,"layout_prediction");
	ros::NodeHandle nh;

    // Define system ...
    System * system = new System (nh);
    Graph * graph = new Graph ();

    // .. and subsystem
    WallDetector * wall_detector = new WallDetector (*system, *graph);
    Optimizer * optimizer = new Optimizer (*system, *graph);

    // initialize threads
    std::thread wall_detector_thread (&WallDetector::run, wall_detector);
    std::thread optimizer_thread (&Optimizer::run, optimizer);

    // Get action data
	ros::Subscriber sub_odom = nh.subscribe ("action", 1, &System::readActionData, system);

    // Get sensors data
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub (nh, "cloud", 1);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub (nh, "rgb", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub (nh, "depth", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odometry_sub (nh, "odometry", 1);

    typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::Image, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync (MySyncPolicy (10), cloud_sub, rgb_sub, depth_sub, odometry_sub);
    sync.registerCallback (boost::bind (&System::readSensorsData, system, _1, _2, _3, _4));

    ros::spin();

	return 0;
}

