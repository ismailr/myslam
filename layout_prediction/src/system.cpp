#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <pr2_mechanism_controllers/BaseOdometryState.h>

#include "layout_prediction/system.h"
#include "layout_prediction/wall_detector.h"
#include "layout_prediction/wall.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/frame.h"
#include "layout_prediction/optimizer.h"
#include "layout_prediction/graph.h"
#include "layout_prediction/tracker.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

System::System(ros::NodeHandle nh):_rosnodehandle (nh)
{
    _pub_marker = _rosnodehandle.advertise<visualization_msgs::Marker> ("line_strip",1);
	_pub_cloud = _rosnodehandle.advertise<sensor_msgs::PointCloud2> ("filtered_cloud",1);
	_pub_depth = _rosnodehandle.advertise<sensor_msgs::Image> ("image_depth",1);
	_pub_rgb = _rosnodehandle.advertise<sensor_msgs::Image> ("image_rgb",1);
}

void System::setWallDetector (WallDetector& wall_detector)
{
    _wall_detector = &wall_detector;
};

void System::setOptimizer (Optimizer& optimizer)
{
    _optimizer = &optimizer;
};

void System::setTracker (Tracker& tracker)
{
    _tracker = &tracker;
}

void System::readSensorsData (
        const sensor_msgs::PointCloud2ConstPtr& cloud, 
        const sensor_msgs::ImageConstPtr& rgb,
        const sensor_msgs::ImageConstPtr& depth,
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
    pose->setEstimate (t);

    Frame *frame = new Frame (_cloud, *pose);
    
    std::unique_lock <std::mutex> lock (_framesQueueMutex);
    _framesQueue.push (frame);
    lock.unlock ();

    // Odometry data
    
    // Process pointcloud with _wall_detector

//    WallDetector wall_detector (_pub_cloud);
//    wall_detector.detect (_cloud);
}

void System::readActionData (const pr2_mechanism_controllers::BaseOdometryState::Ptr& action)
{
    std::cout << "vx = " << action->velocity.linear.x << std::endl;
    std::cout << "vy = " << action->velocity.linear.y << std::endl;
    std::cout << "wz = " << action->velocity.angular.z << std::endl;
    std::cout << "=============================================" << std::endl; 
}

template <typename T>
void System::visualize(T& type)
{

}

template <>
void System::visualize<pcl::PointCloud<pcl::PointXYZ>::Ptr> (pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    _pub_cloud.publish (cloud);
}

static int marker_id = 0;
template <>
void System::visualize<Wall> (std::vector<Wall*> walls)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom_combined";

    marker.id = 0; //marker_id++;
    marker.type = visualization_msgs::Marker::LINE_LIST;

    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.a = 1.0;

    for (int i = 0; i < walls.size (); ++i)
    {
        geometry_msgs::Point p;
        geometry_msgs::Point q;

        Eigen::Vector2d _p = walls[i]->p();
        Eigen::Vector2d _q = walls[i]->q();

        p.x = _p(0); p.y = _p(1);
        q.x = _q(0); q.y = _q(1);

        marker.color.r = p.x;
        marker.color.g = p.y;
        marker.color.b = 1.0;

        marker.points.push_back(p);
        marker.points.push_back(q);
    }

    marker.lifetime = ros::Duration();
    _pub_marker.publish(marker);
}
