#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_ros/point_cloud.h>

#include "layout_prediction/wall_detector.h"
#include "layout_prediction/system.h"
#include "layout_prediction/tracker.h"
#include "layout_prediction/settings.h"

using namespace std;
using namespace g2o;
using namespace Eigen;

int main (int argc, char** argv)
{
    const char *fileconfig = "/home/ism/work/code/ros/src/myslam/layout_prediction/src/myslam.cfg";
    myslam::settings::loadConfFile (fileconfig);

	pcl::console::setVerbosityLevel (pcl::console::L_ALWAYS);

	ros::init (argc,argv,"layout_prediction");
	ros::NodeHandle nh;

    Graph2 graph2;
    System2 system2 (nh, graph2);
    WallDetector2 wallDetector2 (system2, graph2);
    Tracker2 tracker2 (system2, graph2);

    // Get sensors data
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub (nh, "cloud", 1);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub (nh, "rgb", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub (nh, "depth", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odometry_sub (nh, "odometry", 1);
    message_filters::Subscriber<nav_msgs::Odometry> action_sub (nh, "action", 1);
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> odomcombined_sub (nh, "odomcombined", 1);

    typedef message_filters::sync_policies::ApproximateTime
        <   sensor_msgs::PointCloud2, 
            sensor_msgs::Image, 
            sensor_msgs::Image, 
            nav_msgs::Odometry, 
//            nav_msgs::Odometry,
            geometry_msgs::PoseWithCovarianceStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync (MySyncPolicy (10),    cloud_sub, 
                                                                            rgb_sub, 
                                                                            depth_sub, 
                                                                            odometry_sub, 
//                                                                            action_sub,
                                                                            odomcombined_sub);

    sync.registerCallback (boost::bind (&System2::readSensorsData, &system2, _1, _2, _3, _4, _5/*, _6*/));

    ros::spin();
	return 0;
}
