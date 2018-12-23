#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "myslam_mit/slam.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

#include <thread>

int main (int argc, char** argv)
{
//	pcl::console::setVerbosityLevel (pcl::console::L_ALWAYS);

	ros::init (argc,argv,"layout_prediction");
	ros::NodeHandle nh;

	MYSLAM::Slam slam (nh);

	// Get sensors data
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub (nh, "cloud", 1);
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub (nh, "rgb", 1);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub (nh, "depth", 1);
	message_filters::Subscriber<nav_msgs::Odometry> odometry_sub (nh, "odometry", 1);
	message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bb_sub (nh, "box", 1);

	typedef message_filters::sync_policies::ApproximateTime
	<   sensor_msgs::PointCloud2, 
	    sensor_msgs::Image, 
	    sensor_msgs::Image, 
	    nav_msgs::Odometry,
	    darknet_ros_msgs::BoundingBoxes > MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync (MySyncPolicy (100),   cloud_sub, 
									    rgb_sub, 
									    depth_sub, 
									    odometry_sub,
									    bb_sub
									    );

	sync.registerCallback (boost::bind (&MYSLAM::Slam::callback, &slam, _1, _2, _3, _4, _5));

    std::thread t1 (&MYSLAM::Slam::thread1, &slam);
    std::thread t2 (&MYSLAM::Slam::thread2, &slam);

	ros::spin();

    t1.join();
    t2.join();

	return 0;
}
