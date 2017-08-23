#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <pr2_mechanism_controllers/BaseOdometryState.h>

#include "layout_prediction/system.h"
#include "layout_prediction/tracker.h"
#include "layout_prediction/wall_detector.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/frame.h"
#include "layout_prediction/optimizer.h"
#include "layout_prediction/graph.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

unsigned long int System::_frameId = 0;

System::System(ros::NodeHandle nh):_rosnodehandle (nh)
{
    _pub_marker = _rosnodehandle.advertise<visualization_msgs::Marker> ("line_strip",1);
	_pub_cloud = _rosnodehandle.advertise<sensor_msgs::PointCloud2> ("filtered_cloud",1);
	_pub_depth = _rosnodehandle.advertise<sensor_msgs::Image> ("image_depth",1);
}

void System::setTracker (Tracker* tracker)
{
    _tracker = tracker;
    _tracker->attachTo (this);
}

void System::setWallDetector (WallDetector *wall_detector)
{
    _wall_detector = wall_detector;
    _wall_detector->attachTo (this);
};

void System::readSensorsData (
        const sensor_msgs::PointCloud2ConstPtr& cloud, 
//        const sensor_msgs::ImageConstPtr& depth,
        const nav_msgs::OdometryConstPtr& odom)
{
	// Pointcloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*cloud, *_cloud);

    // Getting orientation from odometry 
    // (yaw only since we are working in 2D)
    tf::Quaternion q (
                odom->pose.pose.orientation.x,
                odom->pose.pose.orientation.y,
                odom->pose.pose.orientation.z,
                odom->pose.pose.orientation.w
            );
    tf::Matrix3x3 m (q);
    double roll, pitch, yaw;
    m.getRPY (roll, pitch, yaw);

    double odom_x = odom->pose.pose.position.x;
    double odom_y = odom->pose.pose.position.y;
    double odom_theta = yaw;
    SE2 t (odom_x, odom_y, odom_theta);

    Pose *pose = new Pose ();
    pose->setId (System::_frameId++);
    pose->setEstimate (t);

    Frame *frame = new Frame (_cloud, pose);
    
    std::unique_lock <std::mutex> lock (_wall_detector->_framesQueueMutex);
    _wall_detector->_framesQueue.push (frame);
    lock.unlock ();

    // Odometry data
    
    // Process pointcloud with _wall_detector

//    WallDetector wall_detector (_pub_cloud);
//    wall_detector.detect (_cloud);
}

void System::readActionData (const pr2_mechanism_controllers::BaseOdometryState::Ptr& action)
{
//    std::cout << "vx = " << action->velocity.linear.x << std::endl;
//    std::cout << "vy = " << action->velocity.linear.y << std::endl;
//    std::cout << "wz = " << action->velocity.angular.z << std::endl;
//    std::cout << "=============================================" << std::endl; 
}

void System::visualize()
{

}
