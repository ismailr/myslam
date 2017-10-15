#include <thread>
#include <mutex>
#include <stdlib.h>

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
#include "layout_prediction/pose_measurement.h"
#include "layout_prediction/frame.h"
#include "layout_prediction/optimizer.h"
#include "layout_prediction/graph.h"
#include "layout_prediction/tracker.h"
#include "layout_prediction/helpers.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

System::System(ros::NodeHandle nh, Graph& graph)
    :_rosnodehandle (nh), _graph (&graph), _previousTime (0.0), 
    _currentTime (0.0), _init (true)
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
        const nav_msgs::OdometryConstPtr& odom,
        const nav_msgs::OdometryConstPtr& action)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*cloud, *_cloud);

    _currentTime = ros::Time::now().toSec();
    _currentPosePtr = Pose::Ptr (new Pose);
    _currentPosePtr->setId (_graph->generateIdForVertex());

    if (_init)
    {
        _currentPosePtr->setEstimate (*(estimateFromOdom (odom)));
        _init = false;
    } else {
        double deltaTime = _currentTime - _previousTime;

        // pose by motion model
        double vx = action->pose.pose.position.x;
        double vy = action->pose.pose.position.y;
        double w  = action->pose.pose.orientation.z;
        double x0 = _lastPosePtr->estimate()[0];
        double y0 = _lastPosePtr->estimate()[1];
        double theta0 = _lastPosePtr->estimate()[2];

        double x = x0 + (vx * cos(theta0) + vy * sin(theta0)) * deltaTime;
        double y = y0 + (-vx * sin(theta0) + vy * cos(theta0)) * deltaTime;
        double theta = theta0 + w * deltaTime;

        SE2 m (x, y, theta);
        _currentPosePtr->setEstimate (m);

        // set measurement
        std::tuple<int, int> ids (_lastPosePtr->id(), _currentPosePtr->id());
        _graph->addEdgePose2Pose (ids, *(estimateFromOdom (odom)));
    }

    _graph->addVertex (_currentPosePtr);
    Frame::Ptr framePtr (new Frame (_cloud, _currentPosePtr->id()));
//    if (action->pose.pose.orientation.z < 0.05 && action->pose.pose.orientation.z > -0.05)
//    {
        std::unique_lock <std::mutex> lock (_framesQueueMutex);
        _framesQueue.push (framePtr);
        lock.unlock ();
//    }

    _previousTime = _currentTime;
    _lastPosePtr = _currentPosePtr;
}

template <typename T> void System::visualize(T& type)
{

}

template <> void System::visualize<pcl::PointCloud<pcl::PointXYZ>::Ptr> (pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    _pub_cloud.publish (cloud);
}

static int marker_id = 0;
template <>
void System::visualize<Wall::Ptr> (std::vector<Wall::Ptr> walls)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom_combined";

    marker.id = /*0;*/ marker_id++;
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

SE2* System::estimateFromOdom (const nav_msgs::OdometryConstPtr& odom)
{
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
    SE2* t = new SE2 (odom_x, odom_y, odom_theta);
    return t;
}

int System2::_framecounter = 1;
System2::System2(ros::NodeHandle nh, Graph2& graph)
    :_init(true),_prevTime(0.0),_curTime(0.0),
    _rosnodehandle (nh), _graph (&graph)
{
}

void System2::readSensorsData (
        const sensor_msgs::PointCloud2ConstPtr& cloud, 
        const sensor_msgs::ImageConstPtr& rgb,
        const sensor_msgs::ImageConstPtr& depth,
        const nav_msgs::OdometryConstPtr& odom,
        const nav_msgs::OdometryConstPtr& action)
{
    // transform pointcloud data from sensor frame to robot frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*cloud, *_cloud);
	tf::TransformListener listener;
    pcl_ros::transformPointCloud ("/base_link", *_cloud, *_cloud, listener);

    Pose2::Ptr pose = _tracker->trackPose (odom, action, _init);
    _wallDetector->detect (pose, _cloud);

    if (System2::_framecounter % 3 == 0)
    {
        std::cout << "READY TO LOCAL OPTIMIZE ... " << std::endl;
        _localMapper->local_optimize();
    }

    System2::_framecounter++;

    if (_init == true) _init = false;
}

