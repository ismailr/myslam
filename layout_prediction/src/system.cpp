#include <thread>
#include <mutex>
#include <stdlib.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pr2_mechanism_controllers/BaseOdometryState.h>

#include "layout_prediction/system.h"
#include "layout_prediction/wall_detector.h"
#include "layout_prediction/wall.h"
#include "layout_prediction/pose.h"
#include "layout_prediction/pose_measurement.h"
#include "layout_prediction/graph.h"
#include "layout_prediction/tracker.h"
#include "layout_prediction/helpers.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

static int marker_id = 0;
int System2::_framecounter = 1;

System2::System2(ros::NodeHandle nh, Graph2& graph)
    :_init(true),_prevTime(0.0),_curTime(0.0),
    _rosnodehandle (nh), _graph (&graph)
{
    _listener = new tf::TransformListener;
    _pub_marker = _rosnodehandle.advertise<visualization_msgs::Marker> ("line_strip",1);
    _pub_marker2 = _rosnodehandle.advertise<visualization_msgs::Marker> ("gradient",1);
    _pub_marker3 = _rosnodehandle.advertise<visualization_msgs::Marker> ("localine",1);
	_pub_cloud = _rosnodehandle.advertise<sensor_msgs::PointCloud2> ("filtered_cloud",1);
	_pub_depth = _rosnodehandle.advertise<sensor_msgs::Image> ("image_depth",1);
	_pub_rgb = _rosnodehandle.advertise<sensor_msgs::Image> ("image_rgb",1);
	_pub_odom = _rosnodehandle.advertise<nav_msgs::Odometry> ("myodom",1);

    _graph->setSystem (this);
}

template <typename T> void System2::visualize(T& type)
{

}

template <> void System2::visualize<pcl::PointCloud<pcl::PointXYZ>::Ptr> (pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    _pub_cloud.publish (cloud);
}

template <>
void System2::visualize<nav_msgs::OdometryConstPtr> (nav_msgs::OdometryConstPtr& odom)
{
//    nav_msgs::Odometry _odom;
//    _odom = *odom;
    _pub_odom.publish (*odom);
}

template <>
void System2::visualize<Wall3::Ptr> (std::vector<Wall3::Ptr> walls)
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

        Eigen::Vector2d wp = walls[i]->getp(); 
        Eigen::Vector2d wq = walls[i]->getq();

        p.x = wp.x();
        p.y = wp.y();
        q.x = wq.x();
        q.y = wq.y();

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker.points.push_back(p);
        marker.points.push_back(q);
    }

    marker.lifetime = ros::Duration();
    _pub_marker.publish(marker);
}

template <>
void System2::visualize<Wall3::Ptr> (Wall3::Ptr& w)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";

    marker.id = 0;// marker_id++;
    marker.type = visualization_msgs::Marker::LINE_LIST;

    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.a = 1.0;

    geometry_msgs::Point p;
    geometry_msgs::Point q;

    Eigen::Vector2d wp = w->getp();
    Eigen::Vector2d wq = w->getq();

    p.x = wp.x();
    p.y = wp.y();
    q.x = wq.x();
    q.y = wq.y();

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.points.push_back(p);
    marker.points.push_back(q);

    marker.lifetime = ros::Duration();
    _pub_marker.publish(marker);
}

template <>
void System2::visualize<Wall2::Ptr> (Wall2::Ptr& w)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom_combined";

    marker.id = 0; //marker_id++;
    marker.type = visualization_msgs::Marker::LINE_LIST;

    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.a = 1.0;

    geometry_msgs::Point p;
    geometry_msgs::Point q;

    double rho = w->rho();
    double theta = w->theta();

    p.x = 0.0; p.y = 0.0;
    q.x = rho * cos (theta);
    q.y = rho * sin (theta);

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.points.push_back(p);
    marker.points.push_back(q);

    marker.lifetime = ros::Duration();
    _pub_marker.publish(marker);
}

template <>
void System2::visualize2<Wall2::Ptr> (Wall2::Ptr& w)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom_combined";

    marker.id = 0; // marker_id++;
    marker.type = visualization_msgs::Marker::LINE_LIST;

    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.a = 1.0;

    geometry_msgs::Point p;
    geometry_msgs::Point q;

    double rho = w->rho();
    double theta = w->theta();

    double m;
    theta == M_PI/2 ? m = 100000000000 : m = -1/tan(theta);

    p.x = rho * cos (theta);
    p.y = rho * sin (theta);
    q.x = 1.0;
    q.y = m * q.x;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.points.push_back(p);
    marker.points.push_back(q);

    marker.lifetime = ros::Duration();
    _pub_marker.publish(marker);
}

void System2::visualize_grad (double m, double c)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";

    marker.id = 0; //marker_id++;
    marker.type = visualization_msgs::Marker::LINE_LIST;

    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.a = 1.0;

    geometry_msgs::Point p;
    geometry_msgs::Point q;

    p.x = 0.0; p.y = c;
    m == 0.0 ? q.x = 0.0 : q.x = -c/m; 
    q.y = 0.0;

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    marker.points.push_back(p);
    marker.points.push_back(q);

    marker.lifetime = ros::Duration();
    _pub_marker2.publish(marker);
}

void System2::visualize_rho (double x, double y)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom_combined";

    marker.id = 0; //marker_id++;
    marker.type = visualization_msgs::Marker::LINE_LIST;

    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.a = 1.0;

    geometry_msgs::Point p;
    geometry_msgs::Point q;

    p.x = 0.0; p.y = 0.0;
    q.x = x;
    q.y = y;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    marker.points.push_back(p);
    marker.points.push_back(q);

    marker.lifetime = ros::Duration();
    _pub_marker3.publish(marker);
}
static int frame = 0;
void System2::readSensorsData (
        const sensor_msgs::PointCloud2ConstPtr& cloud, 
        const sensor_msgs::ImageConstPtr& rgb,
        const sensor_msgs::ImageConstPtr& depth,
        const nav_msgs::OdometryConstPtr& odom,
//        const nav_msgs::OdometryConstPtr& action,
        const geometry_msgs::PoseWithCovarianceStampedConstPtr& odomcombined)
{
    // transform pointcloud data from sensor frame to robot frame
//    std::ofstream myfile;
//    myfile.open ("/home/ism/tmp/time.txt", std::ios::out|std::ios::app);
//    myfile << "TIME " << ros::Time::now() << std::endl;
//    myfile.close();

    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*cloud, *_cloud);

    try {
        _listener->waitForTransform ("/openni_rgb_optical_frame", "/base_link", ros::Time::now(), ros::Duration(10.0));
        pcl_ros::transformPointCloud ("/base_link", *_cloud, *_cloud, *_listener);
    } 
    catch (tf::TransformException &ex) {
        ROS_ERROR ("%s", ex.what());
    }
//
//    Eigen::Affine3f t = Eigen::Affine3f::Identity();
//    t.translation() << 0.0, 0.0, 0.0;
//    t.rotate (Eigen::AngleAxisf (M_PI/2, Eigen::Vector3f::UnitZ()));
//    pcl::transformPointCloud (*_cloud, *_cloud, t);

    visualize<pcl::PointCloud<pcl::PointXYZ>::Ptr>(_cloud);

    Pose2::Ptr pose = _tracker->trackPose (odom, odom, odomcombined, _init);
    std::set<int> walls;
    _wallDetector->detect (pose, _cloud, walls);

    _currentOptimization[pose->id()] = walls;

    if (System2::_framecounter % 10 == 0)
//    if (System2::_framecounter == 50)
    {
        _graph->localOptimize(_init, _currentOptimization);
//        _graph->localOptimize(_init);
        _currentOptimization.clear();
//        _currentOptimization[pose->id()] = walls;
    }

    System2::_framecounter++;

    if (_init == true) _init = false;
//    _pub_odom.publish (odom);
}

