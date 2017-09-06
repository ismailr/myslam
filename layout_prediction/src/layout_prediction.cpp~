#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>

#include "layout_prediction/settings.h"
#include "layout_prediction/wall_detector.h"
#include "layout_prediction/system.h"
#include "layout_prediction/optimizer.h"
#include "layout_prediction/tracker.h"


int main (int argc, char** argv)
{
	pcl::console::setVerbosityLevel (pcl::console::L_ALWAYS);

	ros::init (argc,argv,"layout_prediction");
	ros::NodeHandle nh;

    // Define system ...
    System::Ptr systemPtr (new System (nh));
    Graph::Ptr graphPtr (new Graph);
//    auto systemPtr = std::make_shared<System> (nh);
//    auto graphPtr = std::make_shared<Graph> ();

    // .. and subsystem
    auto wallDetectorPtr = std::make_shared<WallDetector> (*systemPtr, *graphPtr);
    auto optimizerPtr = std::make_shared<Optimizer> (*systemPtr, *graphPtr);
    auto trackerPtr = std::make_shared<Tracker> (*systemPtr, *graphPtr);

    // initialize threads
    std::thread wall_detector_thread (&WallDetector::run, wallDetectorPtr);
    std::thread optimizer_thread (&Optimizer::run, optimizerPtr);
    std::thread tracker_thread (&Tracker::run, trackerPtr);


    // Get sensors data
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub (nh, "cloud", 1);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub (nh, "rgb", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub (nh, "depth", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odometry_sub (nh, "odometry", 1);
    message_filters::Subscriber<nav_msgs::Odometry> action_sub (nh, "action", 1);

    typedef message_filters::sync_policies::ApproximateTime
        <   sensor_msgs::PointCloud2, 
            sensor_msgs::Image, 
            sensor_msgs::Image, 
            nav_msgs::Odometry, 
            nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync (MySyncPolicy (10),    cloud_sub, 
                                                                            rgb_sub, 
                                                                            depth_sub, 
                                                                            odometry_sub, 
                                                                            action_sub);

    sync.registerCallback (boost::bind (&System::readSensorsData, systemPtr, _1, _2, _3, _4, _5));

    ros::spin();

	return 0;
}

